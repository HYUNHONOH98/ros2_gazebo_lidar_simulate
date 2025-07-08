import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

def transform_to_matrix(transform: TransformStamped):
    """TransformStamped → 4x4 변환 행렬"""
    t = transform.transform.translation
    q = transform.transform.rotation

    translation = np.array([t.x, t.y, t.z])
    rotation = R.from_quat([q.x, q.y, q.z, q.w])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation.as_matrix()
    matrix[:3, 3] = translation
    return matrix

def matrix_to_transform(matrix: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    """4x4 행렬 → TransformStamped 메시지"""
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child

    translation = matrix[:3, 3]
    rotation = R.from_matrix(matrix[:3, :3]).as_quat()

    t.transform.translation.x = float(translation[0])
    t.transform.translation.y = float(translation[1])
    t.transform.translation.z = float(translation[2])
    t.transform.rotation.x = float(rotation[0])
    t.transform.rotation.y = float(rotation[1])
    t.transform.rotation.z = float(rotation[2])
    t.transform.rotation.w = float(rotation[3])

    return t

class CameraInitTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_init_tf_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(0.5, self.publish_transform)

    def publish_transform(self):
        try:
            # world → body (로봇의 실제 위치)
            t_world_body = self.tf_buffer.lookup_transform('world', 'body', rclpy.time.Time())
            # camera_init → body (SLAM의 상대 위치)
            t_caminit_body = self.tf_buffer.lookup_transform('camera_init', 'body', rclpy.time.Time())

            # 행렬로 변환
            T_world_body = transform_to_matrix(t_world_body)
            T_caminit_body = transform_to_matrix(t_caminit_body)
            T_body_caminit = np.linalg.inv(T_caminit_body)

            # 계산: T_world_caminit = T_world_body × T_body_caminit
            T_world_caminit = np.matmul(T_world_body, T_body_caminit)

            # TF 메시지로 변환 및 퍼블리시
            tf_msg = matrix_to_transform(T_world_caminit, 'world', 'camera_init', self.get_clock().now().to_msg())
            self.br.sendTransform(tf_msg)
            self.get_logger().info('Published world → camera_init')

        except Exception as e:
            self.get_logger().warn(f'Transform 계산 실패: {e}')

def main():
    rclpy.init()
    node = CameraInitTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()