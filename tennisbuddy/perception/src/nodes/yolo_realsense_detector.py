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


def _median_depth(depth_m: np.ndarray, cx: int, cy: int, k: int = 5) -> float:
    """
    Exact same logic as median_depth(depth_m, cx, cy, k) in detectobj.py.
    depth_m: float32 meters
    """
    h, w = depth_m.shape

    x0 = max(cx - k // 2, 0)
    y0 = max(cy - k // 2, 0)
    x1 = min(cx + k // 2 + 1, w)
    y1 = min(cy + k // 2 + 1, h)

    vals = depth_m[y0:y1, x0:x1]
    vals = vals[vals > 0]

    return float(np.median(vals)) if vals.size else float("nan")


class YoloImageDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_image_detector")

        # ---------- Parameters ----------
        default_model_path = os.path.join(
            get_package_share_directory("tennisbuddy_perception"),
            "resource",
            "best.pt",
        )

        self.declare_parameter("model", default_model_path)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("imgsz", 640)

        # Use typical RealSense topics as defaults; you can override in launch.
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        # depth_scale to match detectobj.py:
        #   depth_u16 = np.asanyarray(depth_frame.get_data())
        #   depth_m   = depth_u16.astype(np.float32) * depth_scale
        self.declare_parameter("depth_scale", 0.001)

        # GPU-related parameters
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("half", True)

        # ---------- Read params ----------
        self.model_path: str = (
            self.get_parameter("model").get_parameter_value().string_value
        )
        self.conf_thr: float = float(self.get_parameter("conf").value)
        self.imgsz: int = int(self.get_parameter("imgsz").value)
        self.color_topic: str = (
            self.get_parameter("color_topic").get_parameter_value().string_value
        )
        self.depth_topic: str = (
            self.get_parameter("depth_topic").get_parameter_value().string_value
        )
        self.cam_info_topic: str = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.depth_scale: float = float(self.get_parameter("depth_scale").value)

        self.use_gpu: bool = bool(self.get_parameter("use_gpu").value)
        self.device_req: str = (
            self.get_parameter("device").get_parameter_value().string_value
        )
        self.use_half: bool = bool(self.get_parameter("half").value)

        # ---------- Publishers ----------
        self.pub_conf = self.create_publisher(
            Float32MultiArray, "detections/confidences", 10
        )
        self.pub_bbox = self.create_publisher(
            Int32MultiArray, "detections/bboxes", 10
        )
        self.pub_cbox = self.create_publisher(
            Int32MultiArray, "detections/cboxes", 10
        )
        self.pub_depth = self.create_publisher(
            Float32MultiArray, "detections/depth", 10
        )

        # ---------- Subscribers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.bridge = CvBridge()

        self.sub_color = self.create_subscription(
            Image, self.color_topic, self._on_color, qos
        )
        self.sub_depth = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos
        )
        self.sub_caminfo = self.create_subscription(
            CameraInfo, self.cam_info_topic, self._on_caminfo, 1
        )

        # Latest frames / camera info
        self.last_color: Optional[np.ndarray] = None
        self.last_depth: Optional[np.ndarray] = None   # float32, meters
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
            f"- Model path: {self.model_path} (exists={os.path.exists(self.model_path)})\n"
            f"- Color topic: {self.color_topic}\n"
            f"- Depth topic: {self.depth_topic}\n"
            f"- Depth scale: {self.depth_scale}"
        )

        try:
            if torch_cuda:
                torch.backends.cudnn.benchmark = True
                torch.set_float32_matmul_precision("high")
        except Exception:
            pass

        self._init_model_sync()
        self.create_timer(0.05, self._process_frame)

    # ---------- Subscriber callbacks ----------

    def _on_color(self, msg: Image) -> None:
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_color = cv_img

            if not self._seen_color:
                self._seen_color = True
                self.get_logger().info(
                    f"First color frame received on {self.color_topic}, shape={cv_img.shape}"
                )
        except Exception as e:
            self.get_logger().error(f"cv_bridge color conversion failed: {e}")

    def _on_depth(self, msg: Image) -> None:
        """
        Equivalent of:

            aligned = align.process(frames)
            depth_frame = aligned.get_depth_frame()
            depth_u16   = np.asanyarray(depth_frame.get_data())
            depth_m     = depth_u16.astype(np.float32) * depth_scale
        """
        try:
            # This is like np.asanyarray(depth_frame.get_data()) in pyrealsense2
            depth_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )

            if msg.encoding in ("16UC1", "mono16"):
                depth_u16 = np.asanyarray(depth_raw, dtype=np.uint16)
                depth_m = depth_u16.astype(np.float32) * self.depth_scale
            else:
                # Already meters (e.g. 32FC1)
                depth_m = depth_raw.astype(np.float32)

            self.last_depth = depth_m

            if not self._seen_depth:
                self._seen_depth = True
                self.get_logger().info(
                    f"First depth frame received on {self.depth_topic} "
                    f"(encoding={msg.encoding}, shape={depth_m.shape})"
                )
        except Exception as e:
            self.get_logger().error(f"cv_bridge depth conversion failed: {e}")

    def _on_caminfo(self, msg: CameraInfo) -> None:
        try:
            K = msg.k  # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.last_K = (
                float(K[0]),
                float(K[4]),
                float(K[2]),
                float(K[5]),
            )
            if not self._seen_caminfo:
                self._seen_caminfo = True
                self.get_logger().info(
                    f"CameraInfo received from {self.cam_info_topic}: "
                    f"fx={K[0]:.1f}, fy={K[4]:.1f}, cx={K[2]:.1f}, cy={K[5]:.1f}"
                )
        except Exception as e:
            self.get_logger().error(f"CameraInfo parse failed: {e}")

    # ---------- Sync model loading (GPU → CPU fallback) ----------

    def _init_model_sync(self) -> None:
        torch_cuda = torch.cuda.is_available()

        if self.use_gpu and torch_cuda:
            try:
                self._load_and_warmup(device=self.device_req, half=self.use_half)
                self.active_device = self.device_req
                self.active_half = self.use_half
                self.get_logger().info(
                    f"YOLO ready on {self.active_device} (half={self.active_half})"
                )
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

    def _load_and_warmup(self, device: str, half: bool) -> None:
        torch.set_num_threads(1)
        torch.set_num_interop_threads(1)

        self.get_logger().info(f"Loading YOLO model ({device}, half={half})…")

        mdl = YOLO(self.model_path, task="detect")

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

    def _process_frame(self) -> None:
        if self.model is None or self.last_color is None:
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
            self.get_logger().debug("YOLO inference: 0 detections in this frame")
            return

        xyxy = r.boxes.xyxy.detach().to("cpu").numpy().astype(int)
        conf = r.boxes.conf.detach().to("cpu").numpy().astype(np.float32)

        bboxes_flat: list[int] = []
        cboxes_flat: list[int] = []
        confs: list[float] = []
        z_list: list[float] = []

        depth_img = self.last_depth
        K = self.last_K

        for idx, ((x1, y1, x2, y2), c) in enumerate(zip(xyxy, conf)):
            bboxes_flat.extend([int(x1), int(y1), int(x2), int(y2)])
            confs.append(float(c))

            depth_str = "n/a"
            coord_str = ""

            if depth_img is None:
                self.get_logger().warning("No depth frame received.")
            elif K is not None:
                fx, fy, cx, cy = K

                cx_b = (x1 + x2) // 2
                cy_b = (y1 + y2) // 2

                cboxes_flat.extend([int(cx_b), int(cy_b)])

                # Same as: dist = median_depth(depth_m, cx, cy, k=5)
                z_single = _median_depth(depth_img, cx_b, cy_b, k=5)
                z_list.append(z_single)

                self.get_logger().info(
                    f"Depth at ({cx_b},{cy_b}) = {z_single:.3f} m"
                )

                if np.isfinite(z_single) and z_single > 0.0:
                    X = (cx_b - cx) * z_single / fx
                    Y = (cy_b - cy) * z_single / fy

                    depth_str = f"{z_single:.3f} m"
                    coord_str = f", 3D≈({X:.3f}, {Y:.3f}, {z_single:.3f}) m"

                self.get_logger().info(
                    f"Detection {idx}: conf={c:.3f}, "
                    f"bbox=({x1},{y1},{x2},{y2}), depth={depth_str}{coord_str}"
                )

        try:
            if confs:
                self.pub_conf.publish(Float32MultiArray(data=confs))
                self.pub_bbox.publish(Int32MultiArray(data=bboxes_flat))
                self.pub_cbox.publish(Int32MultiArray(data=cboxes_flat))
                self.pub_depth.publish(Float32MultiArray(data=z_list))
        except Exception as e:
            self.get_logger().error("Inference failed: " + str(e))


def main(args=None) -> None:
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
