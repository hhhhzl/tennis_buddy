#!/usr/bin/env python3
import os
import traceback
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import torch
from ultralytics import YOLO


def _median_depth(depth_img: np.ndarray, x: int, y: int, win: int = 5) -> float:
    h, w = depth_img.shape[:2]
    x1 = max(0, x - win // 2)
    y1 = max(0, y - win // 2)
    x2 = min(w, x + win // 2 + 1)
    y2 = min(h, y + win // 2 + 1)
    patch = depth_img[y1:y2, x1:x2]
    vals = np.where(patch > 0, patch.astype(np.float32), np.nan)
    med = np.nanmedian(vals)

    # return patch
    # Log the depth values for debugging
    if np.isnan(med):
        return float("nan")
    
    # Ensure the median depth is valid and positive
    if np.isfinite(med) and med > 0:
        return float(med)
    else:
        return float("nan")


class YoloImageDetector(Node):
    def __init__(self):
        super().__init__("yolo_image_detector")

        # ---------- Parameters ----------
        default_model_path = os.path.join(
            get_package_share_directory('tennisbuddy_perception'),
            'resource', 'best.pt'
        )
        self.declare_parameter("model", default_model_path)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        # GPU-related parameters
        self.declare_parameter("use_gpu", True)         # try CUDA first
        self.declare_parameter("device", "cuda:0")      # which GPU
        self.declare_parameter("half", True)            # fp16 on GPU (if supported)

        # Read params
        self.model_path: str = self.get_parameter("model").get_parameter_value().string_value
        self.conf_thr: float = float(self.get_parameter("conf").value)
        self.imgsz: int = int(self.get_parameter("imgsz").value)
        self.color_topic: str = self.get_parameter("color_topic").get_parameter_value().string_value
        self.depth_topic: str = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.cam_info_topic: str = self.get_parameter("camera_info_topic").get_parameter_value().string_value

        self.use_gpu: bool = bool(self.get_parameter("use_gpu").value)
        self.device_req: str = self.get_parameter("device").get_parameter_value().string_value
        self.use_half: bool = bool(self.get_parameter("half").value)

        # ---------- Publishers ----------
        self.pub_conf = self.create_publisher(Float32MultiArray, "detections/confidences", 10)
        self.pub_bbox = self.create_publisher(Int32MultiArray, "detections/bboxes", 10)
        self.pub_cbox = self.create_publisher(Int32MultiArray, "detections/cboxes", 10)
        self.pub_depth = self.create_publisher(Float32MultiArray, "detections/depth", 10)

        # ---------- Subscribers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.bridge = CvBridge()
        self.sub_color = self.create_subscription(Image, self.color_topic, self._on_color, qos)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self._on_depth, qos)
        self.sub_caminfo = self.create_subscription(CameraInfo, self.cam_info_topic, self._on_caminfo, 1)

        # Latest frames
        self.last_color: Optional[np.ndarray] = None
        self.last_depth: Optional[np.ndarray] = None
        self.last_K: Optional[Tuple[float, float, float, float]] = None  # fx, fy, cx, cy

        self._seen_color = False
        self._seen_depth = False
        self._seen_caminfo = False

        # ---------- YOLO model (sync load, GPU→CPU fallback) ----------
        self.model: Optional[YOLO] = None
        self.active_device: str = "cpu"
        self.active_half: bool = False

        torch_cuda = torch.cuda.is_available()
        self.get_logger().info(
            "Initializing YOLO detector...\n"
            f"- Python: {os.sys.executable}\n"
            f"- NumPy:  {np.__version__}\n"
            f"- Torch:  {torch.__version__} (cuda_available={torch_cuda})\n"
            f"- Ultralytics: YOLO\n"
            f"- Requested device: {'CUDA' if self.use_gpu else 'CPU'} ({self.device_req}) half={self.use_half}\n"
            f"- Model path: {self.model_path} (exists={os.path.exists(self.model_path)}, "
            f"size={os.path.getsize(self.model_path) if os.path.exists(self.model_path) else 'NA'})"
        )

        try:
            if torch_cuda:
                torch.backends.cudnn.benchmark = True
                torch.set_float32_matmul_precision("high")
        except Exception:
            pass

        # Load model synchronously (no threading)
        self._init_model_sync()

        # Frame processing timer (~20 Hz)
        self.create_timer(0.05, self._process_frame)

    # ---------- Subscribers ----------
    def _on_color(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_color = cv_img
            if not self._seen_color:
                self._seen_color = True
                self.get_logger().info(f"First color frame received on {self.color_topic}")
        except Exception as e:
            self.get_logger().error(f"cv_bridge color conversion failed: {e}")

    def _on_depth(self, msg: Image):
        try:
            if msg.encoding in ("16UC1", "mono16"):
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.last_depth = depth.astype(np.uint16)
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.last_depth = depth.astype(np.float32)
            if not self._seen_depth:
                self._seen_depth = True
                self.get_logger().info(f"First depth frame received on {self.depth_topic} (encoding={msg.encoding})")
        except Exception as e:
            self.get_logger().error(f"cv_bridge depth conversion failed: {e}")

    def _on_caminfo(self, msg: CameraInfo):
        try:
            K = msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.last_K = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))
            if not self._seen_caminfo:
                self._seen_caminfo = True
                self.get_logger().info(f"CameraInfo received from {self.cam_info_topic}: fx={K[0]:.1f}, fy={K[4]:.1f}")
        except Exception as e:
            self.get_logger().error(f"CameraInfo parse failed: {e}")

    # ---------- Sync model loading (GPU → CPU fallback) ----------
    def _init_model_sync(self):
        torch_cuda = torch.cuda.is_available()

        # Try GPU first if requested and available
        if self.use_gpu and torch_cuda:
            try:
                self._load_and_warmup(device=self.device_req, half=self.use_half)
                self.active_device = self.device_req
                self.active_half = self.use_half
                self.get_logger().info(f"YOLO ready on {self.active_device} (half={self.active_half})")
                return
            except Exception as e:
                self.get_logger().error(
                    "YOLO CUDA load/warmup failed; falling back to CPU:\n"
                    + "".join(traceback.format_exception(type(e), e, e.__traceback__))
                )

        try:
            self._load_and_warmup(device="cpu", half=False)
            self.active_device = "cpu"
            self.active_half = False
            self.get_logger().info("YOLO ready on CPU")
        except Exception as e:
            self.get_logger().error(
                "YOLO CPU load/warmup failed:\n"
                + "".join(traceback.format_exception(type(e), e, e.__traceback__))
            )

    def _load_and_warmup(self, device: str, half: bool):
        """Load YOLO and run a tiny warmup inference on the chosen device."""
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)

        self.get_logger().info(f"Loading YOLO model ({device}, half={half})…")
        mdl = YOLO(self.model_path, task="detect")
        # Warmup with a blank image
        dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        _ = mdl.predict(
            dummy,
            imgsz=self.imgsz,
            conf=self.conf_thr,
            device=device,
            half=half,
            verbose=True,
            stream=False,
        )
        self.model = mdl

    # ---------- Frame processing (only publish on detections) ----------
    def _process_frame(self):
        if self.model is None:
            # Model failed to load
            return
        if self.last_color is None:
            # No color frames yet
            return

        img = self.last_color
        results = self.model.predict(
            img,
            imgsz=self.imgsz,
            conf=self.conf_thr,
            device=self.active_device,
            half=self.active_half,
            verbose=False,
        )
        if not results:
            return

        r = results[0]

        if r.boxes is None or len(r.boxes) == 0:
            # Inference is running but nothing detected
            self.get_logger().debug("YOLO inference: 0 detections in this frame")
            return

        xyxy = r.boxes.xyxy.detach().to("cpu").numpy().astype(int)
        conf = r.boxes.conf.detach().to("cpu").numpy().astype(np.float32)

        bboxes_flat = []
        cboxes_flat = []
        confs = []

        depth_img = self.last_depth
        K = self.last_K

        for idx, ((x1, y1, x2, y2), c) in enumerate(zip(xyxy, conf)):
            bboxes_flat.extend([int(x1), int(y1), int(x2), int(y2)])
            confs.append(float(c))
            z = []
            depth_str = "n/a"
            coord_str = ""   
            # self.get_logger().info(f"===========================\n {depth_img}")
            if depth_img is None:     
                self.get_logger().warning("No depth frame received.")     
            # if np.any(depth_img == 0):
            #     self.get_logger().warning(f"Depth image contains zero values at {np.where(depth_img == 0)}")
            if depth_img is not None and K is not None:
                fx, fy, cx, cy = K
                xc = (x1 + x2) // 2
                yc = (y1 + y2) // 2
                cboxes_flat.extend([int(xc), int(yc)])
                # self.get_logger().info(
                # f"bbox=({x1},{y1},{x2},{y2}), xc, yc = ({xc}, {yc})"
                # )
                # patch = _median_depth(depth_img, xc, yc, win=5)
                # self.get_logger().info(f"[patch]={patch}")
                # z_single = _median_depth(depth_img, xc, yc, win=10)
                z = _median_depth(depth_img, xc, yc, win=10)
                # z.append(z_single)
                self.get_logger().info(f"Depth={z}")
                if np.isfinite(z) and z > 0:
                    X = (xc - cx) * z / fx
                    Y = (yc - cy) * z / fy
                    depth_str = f"{z:.3f} mm"
                    coord_str = f", 3D≈({X:.3f}, {Y:.3f}, {z:.3f}) mm"
            self.get_logger().info(
                f"Detection {idx}: conf={c:.3f}, "
                f"bbox=({x1},{y1},{x2},{y2}), depth={depth_str}{coord_str}"
            )
        try:
            # Only publish when we have detections
            self.pub_conf.publish(Float32MultiArray(data=confs))
            self.pub_bbox.publish(Int32MultiArray(data=bboxes_flat))
            self.pub_cbox.publish(Int32MultiArray(data = cboxes_flat))
            # self.pub_depth.publish(Float32MultiArray(data = z))

        except Exception as e:
            self.get_logger().error("Inference failed: " + str(e))


# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = YoloImageDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()