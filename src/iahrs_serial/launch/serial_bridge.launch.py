from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Frame id and topic
    frame_id             = DeclareLaunchArgument('frame_id'            , default_value = 'iahrs_imu'            , description = 'frame id of the imu and magnetic field')
    imu_topic            = DeclareLaunchArgument('imu_topic'           , default_value = '/iahrs/imu'           , description = 'imu topic name'                        )
    magnetic_field_topic = DeclareLaunchArgument('magnetic_field_topic', default_value = '/iahrs/magnetic_field', description = 'magnetic field topic name'             )

    # Serial port
    port_path = DeclareLaunchArgument('port_path', default_value = '/dev/ttyUSB0', description = 'Path to the serial port device')
    baud_rate = DeclareLaunchArgument('baud_rate', default_value = '115200'      , description = 'Serial port baud rate'         )

    # IMU options
    remove_gravitational_acceleration = DeclareLaunchArgument('remove_gravitational_acceleration', 
        default_value = 'false', description = 'Remove gravity acceleration at linear acceleration')

    # IAHRS IMU serial driver
    serial_bridge = Node(package = 'iahrs_serial', executable = 'serial_bridge', name = 'iahrs_serial_bridge', output = 'screen',
        parameters=[{
            'frame_id'                         : LaunchConfiguration('frame_id'),
            'imu_topic'                        : LaunchConfiguration('imu_topic'),
            'magnetic_field_topic'             : LaunchConfiguration('magnetic_field_topic'),
            'port_path'                        : LaunchConfiguration('port_path'),
            'baud_rate'                        : LaunchConfiguration('baud_rate'),
            'remove_gravitational_acceleration': LaunchConfiguration('remove_gravitational_acceleration')
        }]
    )

    return LaunchDescription([
        frame_id,
        imu_topic,
        magnetic_field_topic,
        port_path,
        baud_rate,
        remove_gravitational_acceleration,
        serial_bridge
    ])