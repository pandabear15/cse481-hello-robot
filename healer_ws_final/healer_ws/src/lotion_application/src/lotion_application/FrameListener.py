import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):

    def __init__(self, curr_frame, new_frame):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', curr_frame)
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        self.new_frame = new_frame

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        time_period = 1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)
        
        self.initialized = False

    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = self.new_frame

        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.get_logger().info(
                        f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')
        print(trans)
        
def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()