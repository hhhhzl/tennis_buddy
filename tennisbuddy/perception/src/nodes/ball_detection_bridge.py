#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import numpy as np

class BallDetectionBridge(Node):
    def __init__(self):
        super().__init__('ball_detection_bridge')
        
        self.declare_parameter('bbox_topic', '/detections/bboxes')
        self.declare_parameter('conf_topic', '/detections/confidences')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_topic', '/ball_positions')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('max_depth', 5.0)  # max detection distance (meter)
        
        # subscribe
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.sub_bbox = self.create_subscription(
            Int32MultiArray, 
            self.get_parameter('bbox_topic').value,
            self.on_bbox, qos
        )
        self.sub_conf = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('conf_topic').value,
            self.on_conf, qos
        )
        self.sub_caminfo = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.on_caminfo, 1
        )
        self.sub_depth = self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.on_depth, qos
        )
        
        # publish
        self.pub_balls = self.create_publisher(
            PoseArray,
            self.get_parameter('output_topic').value,
            10
        )
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # state
        self.last_bboxes = None
        self.last_confs = None
        self.last_depth = None
        self.last_K = None
        
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        
        self.get_logger().info(
            f'[ball_detection_bridge] Initialized. '
            f'Converting from {self.camera_frame} to {self.target_frame}'
        )
    
    def on_bbox(self, msg: Int32MultiArray):
        self.last_bboxes = msg.data
    
    def on_conf(self, msg: Float32MultiArray):
        self.last_confs = msg.data
    
    def on_depth(self, msg):
        from cv_bridge import CvBridge
        bridge = CvBridge()
        try:
            if msg.encoding in ("16UC1", "mono16"):
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.last_depth = depth.astype(np.uint16)
            else:
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.last_depth = depth.astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
    
    def on_caminfo(self, msg: CameraInfo):
        K = msg.k
        self.last_K = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))  # fx, fy, cx, cy
    
    def _median_depth(self, depth_img, x, y, win=5):
        h, w = depth_img.shape[:2]
        x1 = max(0, x - win // 2)
        y1 = max(0, y - win // 2)
        x2 = min(w, x + win // 2)
        y2 = min(h, y + win // 2)
        patch = depth_img[y1:y2, x1:x2]
        if patch.size == 0:
            return float("nan")
        if patch.dtype == np.uint16:
            vals = np.where(patch > 0, patch.astype(np.float32), np.nan)
            med = np.nanmedian(vals)
            return float(med / 1000.0) if np.isfinite(med) else float("nan")
        vals = np.where(patch > 0, patch.astype(np.float32), np.nan)
        med = np.nanmedian(vals)
        return float(med) if np.isfinite(med) else float("nan")
    
    def process_detections(self):
        if self.last_bboxes is None or self.last_confs is None:
            return
        if self.last_depth is None or self.last_K is None:
            return
        
        # parse bboxes (4 values per bbox: x1, y1, x2, y2)
        bboxes = []
        for i in range(0, len(self.last_bboxes), 4):
            if i + 3 < len(self.last_bboxes):
                bboxes.append([
                    self.last_bboxes[i],
                    self.last_bboxes[i+1],
                    self.last_bboxes[i+2],
                    self.last_bboxes[i+3]
                ])
        
        if len(bboxes) != len(self.last_confs):
            self.get_logger().warn("Bbox count doesn't match conf count")
            return
        
        fx, fy, cx, cy = self.last_K
        ball_poses_camera = []
        
        for bbox, conf in zip(bboxes, self.last_confs):
            if conf < self.min_conf:
                continue
            
            x1, y1, x2, y2 = bbox
            xc = (x1 + x2) // 2
            yc = (y1 + y2) // 2
            
            z = self._median_depth(self.last_depth, xc, yc, win=5)
            
            if not np.isfinite(z) or z <= 0 or z > self.max_depth:
                continue
            
            # calculate 3D coordinates (camera frame)
            X = (xc - cx) * z / fx
            Y = (yc - cy) * z / fy
            
            pose_camera = Pose()
            pose_camera.position.x = float(X)
            pose_camera.position.y = float(Y)
            pose_camera.position.z = float(z)
            pose_camera.orientation.w = 1.0
            
            ball_poses_camera.append(pose_camera)
        
        if not ball_poses_camera:
            return
        
        # transform to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose_array = PoseArray()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = self.get_clock().now().to_msg()
            
            for pose_camera in ball_poses_camera:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.camera_frame
                pose_stamped.header.stamp = pose_array.header.stamp
                pose_stamped.pose = pose_camera
                
                pose_transformed = do_transform_pose(pose_stamped, transform)
                pose_array.poses.append(pose_transformed.pose)
            
            if pose_array.poses:
                self.pub_balls.publish(pose_array)
                self.get_logger().info(
                    f"Published {len(pose_array.poses)} balls to {self.get_parameter('output_topic').value}"
                )
                
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF transform failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing detections: {e}")

def main():
    rclpy.init()
    node = BallDetectionBridge()
    
    # timer to process detections
    node.create_timer(0.1, node.process_detections)  # 10 Hz
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()