import rclpy
from rclpy.node import Node
from autoware_auto_msgs.msg import BoundingBoxArray
from lgsvl_msgs.msg import Detection3DArray

detected_object_ids_iter = iter(range(9999999))


def aw_box_to_lgsvl_detection3d(aw_box):
    aw_position = aw_box.centroid
    aw_orientation = aw_box.orientation
    aw_size = aw_box.size

    # Convert box position
    from geometry_msgs.msg import Pose
    from geometry_msgs.msg import Point
    from geometry_msgs.msg import Quaternion
    resulting_center = Pose(
        position=Point(x=aw_position.x, y=aw_position.y, z=aw_position.z),
        orientation=Quaternion(x=aw_orientation.x, y=aw_orientation.y, z=aw_orientation.z, w=aw_orientation.w))

    # Convert box size
    from geometry_msgs.msg import Vector3
    resulting_size = Vector3(x=aw_size.x, y=aw_size.y, z=aw_size.z)

    # Convert boundary
    from lgsvl_msgs.msg import BoundingBox3D
    simulator_boundary = BoundingBox3D(position=resulting_center, size=resulting_size)

    # Build Detection3D object
    from lgsvl_msgs.msg import Detection3D
    simulator_detection = Detection3D(
        label='AR-TEST',
        score=1.0,
        bbox=simulator_boundary,
        id=next(detected_object_ids_iter))

    return simulator_detection


def build_header(target_frame):
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    import time
    import math
    epoch_time = time.time()
    nanoseconds, seconds = math.modf(epoch_time)
    seconds = int(seconds)
    nanoseconds = int(nanoseconds * math.pow(10, 9))
    return Header(
        stamp=Time(sec=seconds, nanosec=nanoseconds),
        frame_id=target_frame)


class Converter(Node):
    def __init__(self):
        super().__init__('converter')
        self.declare_parameter('in_topic', '/aw_clustered')
        self.declare_parameter('out_topic', '/lgsvl_detections')
        self.declare_parameter('target_frame', 'base_link')
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            self.get_parameter("in_topic").get_parameter_value().string_value,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Detection3DArray,
            self.get_parameter("out_topic").get_parameter_value().string_value,
            10)

    def listener_callback(self, msg):
        simulator_array = Detection3DArray(
            header=build_header(self.get_parameter("target_frame").get_parameter_value().string_value))
        simulator_array.detections = [aw_box_to_lgsvl_detection3d(box) for box in msg.boxes]
        self.publisher.publish(simulator_array)


def main(args=None):
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
