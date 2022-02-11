"""Launch file for Epson A352 accl_node for epson_accl_ros2_driver ROS2 package"""

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name="serial_port",
            # Used for UART interface only
            # When using SPI interface, this is ignored
            default_value="/dev/ttyUSB0",
            description="Serial port name"),
        DeclareLaunchArgument(
            name="frame_id",
            default_value="imu_link",
            description="imu message frame_id field"),
        DeclareLaunchArgument(
            name="imu_topic",
            default_value="/epson_accl/data_raw",
            description="topic name for publishing imu messages."),
        DeclareLaunchArgument(
            name="burst_polling_rate",
            default_value="4000.0",
            description="Set to atleast 2x the output rate of IMU. Should not need to change."),
        DeclareLaunchArgument(
            name="mesmod_sel",
            default_value="0",
            description="Selects to enable reduced noise floor"),
        DeclareLaunchArgument(
            name="temp_stabil",
            default_value="0",
            description="Selects to enable temperature stabilization against thermal shock"),
        DeclareLaunchArgument(
            name="ext_sel",
            default_value="0",
            description="Selects to enable external trigger function on EXT pin"),
        DeclareLaunchArgument(
            name="ext_pol",
            default_value="0",
            description="When external trigger is enabled, selects falling edge trigger on EXT pin"),
        DeclareLaunchArgument(
            name="dout_rate",
            # Refer to datasheet for valid combination of output rate vs filter setting
            # value: output rate (Hz)
            # 2: 1000 
            # 3: 500 
            # 4: 200 
            # 5: 100 
            # 6: 50 
            default_value="4",
            description="Sets data output rate of ACCL"),
        DeclareLaunchArgument(
            name="filter_sel",
            # Refer to datasheet for valid combination of output rate vs filter setting
            # value: Filter Setting
            # 1: FIR Kaiser filter TAP=64 and fc=83
            # 2: FIR Kaiser filter TAP=64 and fc=220
            # 3: FIR Kaiser filter TAP=128 and fc=36
            # 4: FIR Kaiser filter TAP=128 and fc=110
            # 5: FIR Kaiser filter TAP=128 and fc=350
            # 6: FIR Kaiser filter TAP=512 and fc=9
            # 7: FIR Kaiser filter TAP=512 and fc=16
            # 8: FIR Kaiser filter TAP=512 and fc=60
            # 9: FIR Kaiser filter TAP=512 and fc=210
            # 10: FIR Kaiser filter TAP=512 and fc=460
            # 12: User-Defined FIR filter TAP=4
            # 13: User-Defined FIR filter TAP=64
            # 14: User-Defined FIR filter TAP=128
            # 15: User-Defined FIR filter TAP=512
            default_value="8",
            description="Sets the ACCL filter"),
        launch_ros.actions.Node(
            package='epson_accl_ros2_driver',
            node_executable='accl_node',
            output='screen',
            parameters=[{'__log_level': 'INFO',
                'serial_port': LaunchConfiguration("serial_port"),
                'frame_id': LaunchConfiguration("frame_id"),
                'imu_topic': LaunchConfiguration("imu_topic"),
                'burst_polling_rate': LaunchConfiguration("burst_polling_rate"),
                'mesmod_sel': LaunchConfiguration("mesmod_sel"),
                'temp_stabil': LaunchConfiguration("temp_stabil"),
                'ext_sel': LaunchConfiguration("ext_sel"),          
                'ext_pol': LaunchConfiguration("ext_pol"),
                'dout_rate': LaunchConfiguration("dout_rate"),
                'filter_sel': LaunchConfiguration("filter_sel"),
            }])
    ])
