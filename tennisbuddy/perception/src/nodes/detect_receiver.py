
#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
from vision_msgs.msg import Detection2DArray, Detection3DArray

class DetectionReceiver(Node):
    def __init__(self):
        super().__init__('detection_receiver')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_json = self.create_subscription(String, '/detections/json', self.cb_json, 10)
        self.sub_conf = self.create_subscription(Float32MultiArray, '/detections/confidences', self.cb_conf, 10)
        self.sub_bbox = self.create_subscription(Int32MultiArray, '/detections/bboxes', self.cb_bboxes, 10)
        self.sub_cbox = self.create_subscription(Int32MultiArray, '/detections/cboxes', self.cb_cboxes, 10)
        self.sub_depth = self.create_subscription(Float32MultiArray, '/detections/depth', self.cb_depth, 10)

    def cb_json(self, msg: String):
        try:
            data = json.loads(msg.data)
            dets = data.get('detections', [])
            self.get_logger().info(f"/detections/json: {len(dets)} detections")
        except Exception as e:
            self.get_logger().error(f"JSON parse failed: {e}")

    def cb_conf(self, msg: Float32MultiArray):
        self.get_logger().info(f"/detections/confidences: {list(msg.data)}")

    def cb_bboxes(self, msg: Int32MultiArray):
        vals = list(msg.data)
        boxes = [vals[i:i+4] for i in range(0, len(vals), 4)]
        self.get_logger().info(f"/detections/bboxes: {boxes}")

    def cb_cboxes(self, msg: Int32MultiArray):
        vals = list(msg.data)
        boxes = [vals[i:i+2] for i in range(0, len(vals), 2)]
        self.get_logger().info(f"/detections/cboxes: {boxes}")

    def cb_depth(self, msg: Float32MultiArray):
        self.get_logger().info(f"/detections/depth: {list(msg.data)}")

def main():
    rclpy.init()
    node = DetectionReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
