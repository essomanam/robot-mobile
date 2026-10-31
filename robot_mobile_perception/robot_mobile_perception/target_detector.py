"""Nœud de détection de la cible colorée : images couleur + profondeur -> position 3D.

Entrées : /camera/color/image_raw + /camera/depth/image_raw (même instant),
          /camera/color/camera_info
Sorties :
  /target/detections   vision_msgs/Detection3DArray, publié pour CHAQUE image (vide si la cible
                       n'est pas vue : l'absence de cible est une information explicite)
  /target/debug_image  image annotée (publiée seulement si quelqu'un l'écoute)

    ros2 run robot_mobile_perception target_detector --ros-args \
        --params-file <bringup>/config/perception/target_detector.yaml
"""

import cv2
from cv_bridge import CvBridge
from message_filters import Subscriber, TimeSynchronizer
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_mobile_perception.color_detection import (blob_position, color_mask,
                                                     depth_in_meters, largest_blob)
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


class TargetDetector(Node):

    def __init__(self):
        super().__init__('target_detector')
        self.class_id = self.declare_parameter('class_id', 'red_target').value
        self.hue_ranges = list(self.declare_parameter('hue_ranges', [0, 10, 170, 179]).value)
        self.s_min = self.declare_parameter('saturation_min', 100).value
        self.v_min = self.declare_parameter('value_min', 60).value
        self.kernel = self.declare_parameter('kernel_size', 5).value
        self.min_area = self.declare_parameter('min_area_px', 150).value
        self.max_depth = self.declare_parameter('max_depth', 8.0).value

        self.bridge = CvBridge()
        self.info = None
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, 10)
        color = Subscriber(self, Image, '/camera/color/image_raw')
        depth = Subscriber(self, Image, '/camera/depth/image_raw')
        self.sync = TimeSynchronizer([color, depth], 10)
        self.sync.registerCallback(self.on_images)
        self.pub = self.create_publisher(Detection3DArray, '/target/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/target/debug_image', 1)

    def on_info(self, msg):
        self.info = msg

    def on_images(self, color_msg, depth_msg):
        if self.info is None:
            return
        out = Detection3DArray()
        out.header = color_msg.header
        bgr = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        blob = largest_blob(color_mask(bgr, self.hue_ranges, self.s_min, self.v_min,
                                       self.kernel), self.min_area)
        position = None
        if blob is not None:
            k = self.info.k
            depth = depth_in_meters(self.bridge.imgmsg_to_cv2(depth_msg))
            position = blob_position(blob, depth, k[0], k[4], k[2], k[5], self.max_depth)
        if position is not None:
            x, y, z = position
            det = Detection3D()
            det.header = color_msg.header
            det.id = self.class_id
            det.bbox.center.position.x, det.bbox.center.position.y = x, y
            det.bbox.center.position.z = z
            det.bbox.center.orientation.w = 1.0
            det.bbox.size.x = blob.width * z / k[0]   # largeur apparente (m)
            det.bbox.size.y = blob.height * z / k[4]  # hauteur apparente (m)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = self.class_id
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose = det.bbox.center
            det.results.append(hypothesis)
            out.detections.append(det)
        self.pub.publish(out)
        if self.debug_pub.get_subscription_count() > 0:
            self.publish_debug(bgr, blob, position, color_msg.header)

    def publish_debug(self, bgr, blob, position, header):
        if blob is not None:
            cv2.rectangle(bgr, (blob.x, blob.y), (blob.x + blob.width, blob.y + blob.height),
                          (0, 255, 0), 2)
            cv2.circle(bgr, (int(blob.u), int(blob.v)), 5, (255, 0, 0), -1)
            if position is not None:
                cv2.putText(bgr, f'{position[2]:.2f} m', (blob.x, max(15, blob.y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        msg = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
