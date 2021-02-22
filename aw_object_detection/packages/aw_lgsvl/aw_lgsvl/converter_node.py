import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from autoware_auto_msgs.msg import BoundingBoxArray
from lgsvl_msgs.msg import Detection3DArray
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

detected_object_ids_iter = iter(range(9999999))


def extract_centroid_and_quaternion_from_ego_pose(ego_pose):
    ego_position_vector = [ego_pose.position.x, ego_pose.position.y, ego_pose.position.z]
    ego_orientation = ego_pose.orientation
    ego_quaternion = Quaternion(ego_orientation.w, ego_orientation.x, ego_orientation.y, ego_orientation.z)
    return {
        "ego_position": ego_pose,
        "ego_position_vector": ego_position_vector,
        "ego_quaternion": ego_quaternion
    }


def aw_box_to_lgsvl_detection3d(aw_box, ego_data, frame_header):
    from lgsvl_msgs.msg import Detection3D
    simulator_detection = Detection3D()
    simulator_detection.label = 'AR_TEST'
    simulator_detection.score = 1.0
    simulator_detection.id = next(detected_object_ids_iter)
    simulator_detection.header = frame_header

    from lgsvl_msgs.msg import BoundingBox3D
    from geometry_msgs.msg import Vector3
    simulator_boundary = BoundingBox3D()
    simulator_boundary.size = Vector3(x=aw_box.size.x, y=aw_box.size.y, z=aw_box.size.z)

    if aw_box.vehicle_label != 2:
        aw_position_vector = [aw_box.centroid.x, aw_box.centroid.y, aw_box.centroid.z]
        aw_quaternion = Quaternion(
            aw_box.orientation.w,
            aw_box.orientation.x,
            aw_box.orientation.y,
            aw_box.orientation.z)

        # Converted centroid to map space
        rotated_centroid = ego_data['ego_quaternion'].rotate(aw_position_vector)
        shifted_centroid = [0, 0, 0]
        for i in range(len(aw_position_vector)):
            shifted_centroid[i] = rotated_centroid[i] + ego_data['ego_position_vector'][i]

        # Convert box rotation to map space

        # orientation of gps sensor needs to be
        # rotated by 90 degree around Z-axis.
        z_rotate_90_degrees = Quaternion(axis=[0.0, 0.0, 1.0], degrees=90)
        resulting_quaternion = z_rotate_90_degrees * ego_data['ego_quaternion'] * aw_quaternion

        # Convert box position
        from geometry_msgs.msg import Pose
        from geometry_msgs.msg import Point
        from geometry_msgs.msg import Quaternion as RosQuaternion
        resulting_center = Pose(
            position=Point(x=shifted_centroid[0], y=shifted_centroid[1], z=shifted_centroid[2]),
            orientation=RosQuaternion(
                x=resulting_quaternion.x,
                y=resulting_quaternion.y,
                z=resulting_quaternion.z,
                w=resulting_quaternion.w
            )
        )

        # Convert boundary
        simulator_boundary.position = resulting_center

        # Build Detection3D object
        simulator_detection.bbox = simulator_boundary

    else:
        # Let simulator to color ego_box
        simulator_detection.label = 'Car'
        simulator_boundary.position = ego_data['ego_position']
        simulator_detection.bbox = simulator_boundary

    return simulator_detection


def build_header(target_frame):
    from builtin_interfaces.msg import Time
    from std_msgs.msg import Header
    import time
    import math
    epoch_time = time.time()
    nanoseconds, seconds = math.modf(epoch_time)
    seconds = int(seconds)
    nanoseconds = int(nanoseconds * 10 ** 9)
    return Header(
        stamp=Time(sec=seconds, nanosec=nanoseconds),
        frame_id=target_frame)


class Converter(Node):

    def __init__(self):
        super().__init__('converter')
        self.declare_parameter('in_topic', '/aw_clustered')
        self.declare_parameter('out_topic', '/lgsvl_detections')
        self.declare_parameter('gps', '/lgsvl_gps')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('show_ego_car', 'false')

        from autoware_auto_msgs.msg import BoundingBox
        self.ego_gps_pose = None
        self.ego_box = BoundingBox()
        self.ego_box.size.x = 5.0
        self.ego_box.size.y = 2.5
        self.ego_box.size.z = 1.5
        self.ego_box.centroid.x = 0.0
        self.ego_box.centroid.y = 0.0
        self.ego_box.centroid.z = 0.0
        self.ego_box.vehicle_label = 2

        self.subscription_boxes = self.create_subscription(
            BoundingBoxArray,
            self.get_parameter("in_topic").get_parameter_value().string_value,
            self.boxes_callback,
            10)
        self.subscription_gps = self.create_subscription(
            Odometry,
            self.get_parameter("gps").get_parameter_value().string_value,
            self.gps_callback,
            10)
        self.publisher = self.create_publisher(
            Detection3DArray,
            self.get_parameter("out_topic").get_parameter_value().string_value,
            10)

    def boxes_callback(self, msg):
        if self.ego_gps_pose is not None:
            self.ego_box.centroid.x = self.ego_gps_pose.position.x
            self.ego_box.centroid.y = self.ego_gps_pose.position.y
            self.ego_box.centroid.z = self.ego_gps_pose.position.z
            msg.boxes.append(self.ego_box)
            frame_header = build_header(self.get_parameter("target_frame").get_parameter_value().string_value)
            simulator_array = Detection3DArray(
                header=frame_header)
            simulator_array.detections = [aw_box_to_lgsvl_detection3d(
                box,
                extract_centroid_and_quaternion_from_ego_pose(self.ego_gps_pose),
                frame_header=frame_header) for box in msg.boxes]
            reprocess_coords_for_lgsvl(simulator_array)
            self.publisher.publish(simulator_array)
        else:
            self.get_logger().info('No ego pose detected')

    def gps_callback(self, msg):
        self.ego_gps_pose = msg.pose.pose


def reprocess_coords_for_lgsvl(detection_3d_array: Detection3DArray):
    original_detections = detection_3d_array.detections
    for original_detection in original_detections:
        original_position = original_detection.bbox.position.position
        original_x = original_position.x
        original_y = original_position.y
        original_z = original_position.z
        new_position = Point(x=-original_y, y=original_z, z=original_x)
        from geometry_msgs.msg import Vector3
        original_size = original_detection.bbox.size
        new_size = Vector3(x=original_size.y, y=original_size.z, z=original_size.x)
        original_detection.bbox.position.position = new_position
        original_detection.bbox.size = new_size
        original_orientation = original_detection.bbox.position.orientation
        from geometry_msgs.msg import Quaternion as RosQuaternion
        new_orientation = RosQuaternion(
            x=-original_orientation.y,
            y=original_orientation.z,
            z=original_orientation.x,
            w=-original_orientation.w
        )
        original_detection.bbox.position.orientation = new_orientation


def main(args=None):
    rclpy.init(args=args)
    converter = Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
