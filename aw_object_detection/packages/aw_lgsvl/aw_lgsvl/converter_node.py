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


def create_detection_array():
    return Detection3DArray(header=build_header())


def build_header():
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
        frame_id='base_link')  # TODO: read frame_id from parameter


class Converter(Node):
    def __init__(self):
        super().__init__('converter')
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            '/robolux/lidar_bounding_boxes',  # TODO: read topic name from parameter
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Detection3DArray,
            '/robolux/lgsvl_detections',  # TODO: read topic name from parameter
            10)

    def listener_callback(self, msg):
        simulator_array = create_detection_array()
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
