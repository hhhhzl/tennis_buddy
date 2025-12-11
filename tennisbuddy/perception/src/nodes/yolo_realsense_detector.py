#!/usr/bin/env python3

import os
import sys
import traceback
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import torch
from ultralytics import YOLO


def _median_depth(depth_m: np.ndarray, cx: int, cy: int, k: int = 5) -> float:
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

        # Default to typical realsense2_camera topics when align_depth:=true
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")

        # For RealSense ROS depth: 16UC1 in mm → scale=0.001 to get meters
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

        # ---------- Ready / status publisher (latched) ----------
        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latch last msg
        )
        self.ready_pub = self.create_publisher(Bool, "yolo_ready", ready_qos)

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
        self.last_color: Optional[np.ndarray] = None      # BGR8
        self.last_depth: Optional[np.ndarray] = None      # float32, meters
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
            "Initializing YOLO detector (ROS + RealSense topics)…\n"
            f"- Python: {sys.executable}\n"
            f"- Model path: {self.model_path} (exists={os.path.exists(self.model_path)})\n"
            f"- Color topic: {self.color_topic}\n"
            f"- Depth topic: {self.depth_topic}\n"
            f"- CameraInfo topic: {self.cam_info_topic}\n"
            f"- Depth scale: {self.depth_scale}\n"
            f"- Torch CUDA available: {torch_cuda}"
        )

        # Set thread configuration early (before any model loading)
        try:
            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
        except Exception:
            # Thread settings may fail if already set, ignore
            pass

        try:
            if torch_cuda:
                # Completely disable cuDNN to avoid "FIND/GET engine" errors on Jetson
                # cuDNN v8/v9 compatibility issues cause CUDNN_STATUS_EXECUTION_FAILED
                # PyTorch will fall back to other CUDA backends (e.g., CUBLAS)
                torch.backends.cudnn.enabled = False
                self.get_logger().info(
                    "cuDNN disabled - using PyTorch default CUDA backend "
                    "(may be slower but more stable on Jetson)"
                )
                torch.set_float32_matmul_precision("high")
        except Exception:
            pass

        # Disable TensorRT and other optimizations to avoid "FIND engine" errors on Jetson
        os.environ.setdefault("YOLO_TENSORRT", "False")
        os.environ.setdefault("YOLO_ORT", "False")  # Disable ONNX Runtime
        os.environ.setdefault("YOLO_COREML", "False")  # Disable CoreML
        # Force PyTorch backend
        os.environ.setdefault("YOLO_ENGINE", "torch")

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
                    f"First color frame received on {self.color_topic}, "
                    f"shape={cv_img.shape}"
                )
        except Exception as e:
            self.get_logger().error(f"cv_bridge color conversion failed: {e}")

    def _on_depth(self, msg: Image) -> None:
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )

            if msg.encoding in ("16UC1", "mono16"):
                depth_u16 = depth_raw.astype(np.uint16)
                depth_m = depth_u16.astype(np.float32) * self.depth_scale
            else:
                # If already in meters (e.g. 32FC1)
                depth_m = depth_raw.astype(np.float32)

            self.last_depth = depth_m

            if not self._seen_depth:
                self._seen_depth = True
                self.get_logger().info(
                    f"First depth frame received on {self.depth_topic}, "
                    f"encoding={msg.encoding}, shape={depth_m.shape}"
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

    # ---------- Ready helper ----------

    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        self.get_logger().info("YOLO model initialized; published yolo_ready = True")

    # ---------- Sync model loading (GPU → CPU fallback) ----------

    def _init_model_sync(self) -> None:
        if torch.cuda.is_available():
            # Try with half precision first
            try:
                self._load_and_warmup(device="cuda:0", half=True)
                self.active_device = "cuda:0"
                self.active_half = True
                self.get_logger().info("YOLO ready on CUDA (half precision)")
                self._publish_ready()
                return
            except Exception as e:
                error_str = str(e)
                self.get_logger().warning(
                    f"[YOLO] CUDA half precision failed: {error_str[:200]}"
                )
                # If FIND engine error, try without half precision
                if "FIND" in error_str or "engine" in error_str.lower():
                    self.get_logger().info(
                        "[YOLO] Retrying CUDA with float32 (no half precision)..."
                    )
                    try:
                        self._load_and_warmup(device="cuda:0", half=False)
                        self.active_device = "cuda:0"
                        self.active_half = False
                        self.get_logger().info("YOLO ready on CUDA (float32)")
                        self._publish_ready()
                        return
                    except Exception as e2:
                        self.get_logger().error(
                            "[YOLO] CUDA float32 also failed, falling back to CPU:\n"
                            + "".join(traceback.format_exception(type(e2), e2, e2.__traceback__))
                        )
                else:
                    self.get_logger().error(
                        "[YOLO] CUDA load/warmup failed, falling back to CPU:\n"
                        + "".join(traceback.format_exception(type(e), e, e.__traceback__))
                    )
        try:
            self._load_and_warmup(device="cpu", half=False)
            self.active_device = "cpu"
            self.active_half = False
            self.get_logger().info("YOLO ready on CPU (float32)")
            self._publish_ready()
        except Exception as e:
            self.get_logger().error(
                "[YOLO] CPU load/warmup failed:\n"
                + "".join(traceback.format_exception(type(e), e, e.__traceback__))
            )

    def _load_and_warmup(self, device: str, half: bool) -> None:
        # Thread settings should be set before any torch operations
        # Skip if already set or if parallel work has started
        try:
            torch.set_num_threads(1)
        except RuntimeError:
            # Threads already set or parallel work started, skip
            pass
        
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # Interop threads already set, skip
            pass

        self.get_logger().info(f"Loading YOLO model ({device}, half={half})…")

        # Load model with explicit backend settings to avoid engine errors
        # The "FIND engine" error typically occurs when ultralytics tries to use
        # TensorRT or other optimized backends that aren't properly configured
        try:
            mdl = YOLO(self.model_path, task="detect")
        except Exception as e:
            if "FIND" in str(e) or "engine" in str(e).lower():
                self.get_logger().warning(
                    f"Model loading with default backend failed: {e}\n"
                    "Trying with explicit PyTorch backend..."
                )
                # Force PyTorch backend
                os.environ["YOLO_ENGINE"] = "torch"
                mdl = YOLO(self.model_path, task="detect")
            else:
                raise

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
        # Need model + color + depth
        if self.model is None or self.last_color is None or self.last_depth is None:
            return

        img = self.last_color
        depth_img = self.last_depth
        K = self.last_K

        # YOLO inference
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

        bboxes_flat: List[int] = []
        cboxes_flat: List[int] = []
        confs: List[float] = []
        z_list: List[float] = []

        for idx, ((x1, y1, x2, y2), c) in enumerate(zip(xyxy, conf)):
            bboxes_flat.extend([int(x1), int(y1), int(x2), int(y2)])
            confs.append(float(c))

            if depth_img is None:
                self.get_logger().warning("No depth frame available.")
                z_single = float("nan")
                z_list.append(z_single)
                continue

            cx_b = (x1 + x2) // 2
            cy_b = (y1 + y2) // 2
            cboxes_flat.extend([int(cx_b), int(cy_b)])

            z_single = _median_depth(depth_img, cx_b, cy_b, k=5)
            z_list.append(z_single)

            if K is not None and np.isfinite(z_single) and z_single > 0.0:
                fx, fy, cx, cy = K
                X = (cx_b - cx) * z_single / fx
                Y = (cy_b - cy) * z_single / fy
                # depth_str = f"{z_single:.3f} m"
                # coord_str = f", 3D≈({X:.3f}, {Y:.3f}, {z_single:.3f}) m"
                # self.get_logger().info(
                #     f"Detection {idx}: conf={c:.3f}, "
                #     f"bbox=({x1},{y1},{x2},{y2}), depth={depth_str}{coord_str}"
                # )

        try:
            if confs:
                self.pub_conf.publish(Float32MultiArray(data=confs))
                self.pub_bbox.publish(Int32MultiArray(data=bboxes_flat))
                self.pub_cbox.publish(Int32MultiArray(data=cboxes_flat))
                self.pub_depth.publish(Float32MultiArray(data=z_list))
        except Exception as e:
            self.get_logger().error("Publishing failed: " + str(e))


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
