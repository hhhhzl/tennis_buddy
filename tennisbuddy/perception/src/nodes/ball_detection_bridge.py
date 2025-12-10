#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import rclpy.time

class BallDetectionBridge(Node):
    def __init__(self):
        super().__init__('ball_detection_bridge')
        
        self.declare_parameter('detection_timeout', 0.5)  # seconds - max age of detection
        self.declare_parameter('time_sync_tolerance', 0.1)  # max time difference between bbox/conf/depth
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
        
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        self.time_sync_tolerance = float(self.get_parameter('time_sync_tolerance').value)

        # Store data with timestamps for time synchronization
        self.last_bbox_data = None  # (data, timestamp)
        self.last_conf_data = None  # (data, timestamp)
        self.last_depth_data = None  # (image, timestamp)
        self.last_K = None
        
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_conf = float(self.get_parameter('min_confidence').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        
        self.get_logger().info(
            f'[ball_detection_bridge] Initialized. '
            f'Converting from {self.camera_frame} to {self.target_frame}, '
            f'timeout={self.detection_timeout}s, sync_tolerance={self.time_sync_tolerance}s'
        )
    
    def on_bbox(self, msg: Int32MultiArray):
        # Use message timestamp if available, otherwise use current time
        if hasattr(msg, 'header') and msg.header.stamp:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        else:
            stamp = self.get_clock().now()
        self.last_bbox_data = (msg.data, stamp)
    
    def on_conf(self, msg: Float32MultiArray):
        # Use message timestamp if available, otherwise use current time
        if hasattr(msg, 'header') and msg.header.stamp:
            stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        else:
            stamp = self.get_clock().now()
        self.last_conf_data = (msg.data, stamp)
    
    def on_depth(self, msg: Image):
        from cv_bridge import CvBridge
        bridge = CvBridge()
        try:
            # Use depth image timestamp - this is critical for TF synchronization!
            depth_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            
            if msg.encoding in ("16UC1", "mono16"):
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_img = depth.astype(np.uint16)
            else:
                depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth_img = depth.astype(np.float32)
            
            self.last_depth_data = (depth_img, depth_stamp)
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
        now = self.get_clock().now()
        
        # Check if we have all required data
        if self.last_bbox_data is None or self.last_conf_data is None or self.last_depth_data is None:
            return
        
        bbox_data, bbox_stamp = self.last_bbox_data
        conf_data, conf_stamp = self.last_conf_data
        depth_img, depth_stamp = self.last_depth_data
        
        # Check if detections are too old
        bbox_age = (now - bbox_stamp).nanoseconds / 1e9
        conf_age = (now - conf_stamp).nanoseconds / 1e9
        depth_age = (now - depth_stamp).nanoseconds / 1e9
        
        if bbox_age > self.detection_timeout or conf_age > self.detection_timeout or depth_age > self.detection_timeout:
            # Publish empty to clear old positions
            pose_array = PoseArray()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = now.to_msg()
            self.pub_balls.publish(pose_array)
            return
        
        # Check time synchronization between bbox, conf, and depth
        # Use depth timestamp as reference (most important for 3D reconstruction)
        time_diffs = [
            abs((bbox_stamp - depth_stamp).nanoseconds / 1e9),
            abs((conf_stamp - depth_stamp).nanoseconds / 1e9),
        ]
        max_diff = max(time_diffs)
        
        if max_diff > self.time_sync_tolerance:
            self.get_logger().debug(
                f"Time sync check failed: bbox-depth={time_diffs[0]:.3f}s, "
                f"conf-depth={time_diffs[1]:.3f}s (tolerance={self.time_sync_tolerance}s)"
            )
            return
        
        if self.last_K is None:
            return
        
        # Parse bboxes (4 values per bbox: x1, y1, x2, y2)
        bboxes = []
        for i in range(0, len(bbox_data), 4):
            if i + 3 < len(bbox_data):
                bboxes.append([bbox_data[i], bbox_data[i+1], bbox_data[i+2], bbox_data[i+3]])
        
        if len(bboxes) != len(conf_data):
            self.get_logger().warn("Bbox count doesn't match conf count")
            return
        
        fx, fy, cx, cy = self.last_K
        ball_poses_camera = []
        
        for bbox, conf in zip(bboxes, conf_data):
            if conf < self.min_conf:
                continue
            
            x1, y1, x2, y2 = bbox
            xc = (x1 + x2) // 2
            yc = (y1 + y2) // 2
            
            z = self._median_depth(depth_img, xc, yc, win=5)
            
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
            pose_array = PoseArray()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = now.to_msg()
            self.pub_balls.publish(pose_array)
            return
        
        # Transform to map frame using depth image timestamp
        try:
            pose_array = PoseArray()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = now.to_msg()
            
            for pose_camera in ball_poses_camera:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.camera_frame
                # CRITICAL: Use depth image timestamp for TF lookup
                # This ensures we use the camera pose at the time of detection
                pose_stamped.header.stamp = depth_stamp.to_msg()
                pose_stamped.pose = pose_camera
                
                try:
                    # Try direct transform first
                    pose_transformed = self.tf_buffer.transform(
                        pose_stamped,
                        self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=2.0)  # Increased timeout for Nav2 resize
                    )
                    pose_array.poses.append(pose_transformed.pose)
                except tf2_ros.TransformException as e:
                    # If direct transform fails, try two-step transform via base_link
                    # This helps when TF tree is temporarily disconnected (e.g., during Nav2 resize)
                    try:
                        # Step 1: camera -> base_link
                        pose_base = self.tf_buffer.transform(
                            pose_stamped,
                            'base_link',
                            timeout=rclpy.duration.Duration(seconds=2.0)
                        )
                        # Step 2: base_link -> map
                        pose_transformed = self.tf_buffer.transform(
                            pose_base,
                            self.target_frame,
                            timeout=rclpy.duration.Duration(seconds=2.0)
                        )
                        pose_array.poses.append(pose_transformed.pose)
                        self.get_logger().debug("Used two-step transform via base_link")
                    except tf2_ros.TransformException as e2:
                        # Final fallback: use latest available transform
                        try:
                            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
                            pose_transformed = self.tf_buffer.transform(
                                pose_stamped,
                                self.target_frame,
                                timeout=rclpy.duration.Duration(seconds=2.0)
                            )
                            pose_array.poses.append(pose_transformed.pose)
                            self.get_logger().debug("Used latest transform as final fallback")
                        except tf2_ros.TransformException as e3:
                            # Check if it's a tree connection issue
                            error_str = str(e3)
                            if "not part of the same tree" in error_str or "unconnected trees" in error_str:
                                self.get_logger().warn(
                                    f"TF tree disconnected - camera frame may not be published. "
                                    f"Ensure robot_state_publisher is running. Error: {e3}"
                                )
                            else:
                                self.get_logger().warn(f"Transform failed after all retries: {e3}")
                            continue
            
            if pose_array.poses:
                self.pub_balls.publish(pose_array)
                self.get_logger().debug(
                    f"Published {len(pose_array.poses)} balls (detection time: {depth_stamp.seconds_nanoseconds()})"
                )
                
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