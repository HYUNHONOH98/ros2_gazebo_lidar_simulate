import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time

class StaticCameraInitPublisher(Node):
    def __init__(self):
        super().__init__('static_camera_init_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        self.camera_init_broadcaster = StaticTransformBroadcaster(self)

        self.initialized = False
        self.timer = self.create_timer(0.5, self.try_publish)

    def try_publish(self):
        if self.initialized:
            return
        try:
            odom_static = TransformStamped()
            odom_static.header.stamp = self.get_clock().now().to_msg()
            odom_static.header.frame_id = 'world'
            odom_static.child_frame_id = 'odom'

            odom_static.transform.translation.x = 0.0
            odom_static.transform.translation.y = 0.0
            odom_static.transform.translation.z = 0.0

            odom_static.transform.rotation.x = 0.0
            odom_static.transform.rotation.y = 0.0
            odom_static.transform.rotation.z = 0.0
            odom_static.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(odom_static)


            cam_static = TransformStamped()
            cam_static.header.stamp = self.get_clock().now().to_msg()
            cam_static.header.frame_id = 'world'
            cam_static.child_frame_id = 'camera_init'

            tf = self.tf_buffer.lookup_transform('odom', 'livox', Time())

            cam_static.transform.translation.x = 0.0
            cam_static.transform.translation.y = 0.0
            cam_static.transform.translation.z = tf.transform.translation.z

            cam_static.transform.rotation.x = 0.0
            cam_static.transform.rotation.y = 0.0
            cam_static.transform.rotation.z = 0.0
            cam_static.transform.rotation.w = 1.0
            self.camera_init_broadcaster.sendTransform(cam_static)

            self.initialized = True  # 한 번만 퍼블리시

        except Exception as e:
            self.get_logger().warn(f'변환 아직 못 가져옴: {e}')

def main():
    rclpy.init()
    node = StaticCameraInitPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()