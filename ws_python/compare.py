import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time
import numpy as np


class TFPrinter(Node):
    def __init__(self):
        super().__init__('tf_printer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.print_transforms)

        self.t1s = np.zeros((1, 7), dtype=np.float32)
        self.t2s = np.zeros((1, 7), dtype=np.float32)

    def print_transforms(self):
        try:
            # 1. camera_init → livox
            trans1 = self.tf_buffer.lookup_transform(
                'camera_init', 'body', Time())
            t1 = trans1.transform
            self.t1s = np.vstack((self.t1s, np.array([
                t1.translation.x, t1.translation.y, t1.translation.z, t1.rotation.x, t1.rotation.y, t1.rotation.z, t1.rotation.w
            ], dtype=np.float32)))
            
        except Exception as e:
            self.get_logger().warn(f'camera_inWit → livox 변환 실패: {e}')

        try:
            # 2. odom → base_link
            trans2 = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time())
            t2 = trans2.transform
            self.t2s = np.vstack((self.t2s, np.array([
                t2.translation.x, t2.translation.y, t2.translation.z, t2.rotation.x, t2.rotation.y, t2.rotation.z, t2.rotation.w
            ], dtype=np.float32)))
        except Exception as e:
            self.get_logger().warn(f'odom → base_link 변환 실패: {e}')


def main():
    try:
        rclpy.init()
        node = TFPrinter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        np.save('/root/ws_python/t1s.npy', node.t1s)
        np.save('/root/ws_python/t2s.npy', node.t2s)
    

if __name__ == '__main__':
    main()