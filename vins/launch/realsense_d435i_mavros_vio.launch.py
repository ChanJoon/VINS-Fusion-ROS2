"""Run VINS-Fusion with D435i stereo+IMU input and forward VIO to MAVROS."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('vins')
    default_config = os.path.join(
        pkg_share, 'config', 'qav250_d435i_px4imu', 'stereo_imu_config.yaml')

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='Path to the VINS-Fusion D435i stereo+IMU config YAML')
    bridge_input_topic_arg = DeclareLaunchArgument(
        'bridge_input_topic',
        default_value='/vins_estimator/odometry',
        description='VINS odometry topic forwarded to MAVROS.')
    bridge_output_topic_arg = DeclareLaunchArgument(
        'bridge_output_topic',
        default_value='/mavros/odometry/out',
        description='MAVROS odometry input topic for PX4 external vision.')
    bridge_max_publish_rate_hz_arg = DeclareLaunchArgument(
        'bridge_max_publish_rate_hz',
        default_value='40.0',
        description='Maximum VIO odometry rate sent to MAVROS. Set 0 for unlimited.')
    publish_odometry_bridge_arg = DeclareLaunchArgument(
        'publish_odometry_bridge',
        default_value='false',
        description='Publish nav_msgs/Odometry to /mavros/odometry/out.')
    publish_vision_pose_bridge_arg = DeclareLaunchArgument(
        'publish_vision_pose_bridge',
        default_value='true',
        description='Publish geometry_msgs/PoseStamped to MAVROS vision_pose plugin.')
    vision_pose_output_topic_arg = DeclareLaunchArgument(
        'vision_pose_output_topic',
        default_value='/mavros/vision_pose/pose',
        description='MAVROS vision_pose PoseStamped input topic.')
    vision_pose_frame_id_arg = DeclareLaunchArgument(
        'vision_pose_frame_id',
        default_value='odom',
        description='frame_id written on PoseStamped messages sent to MAVROS vision_pose.')
    vision_pose_rate_hz_arg = DeclareLaunchArgument(
        'vision_pose_rate_hz',
        default_value='40.0',
        description='Maximum PoseStamped rate sent to MAVROS vision_pose.')
    publish_local_position_path_arg = DeclareLaunchArgument(
        'publish_local_position_path',
        default_value='true',
        description='Accumulate /mavros/local_position/pose as /mavros/local_position/path for RViz.')

    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        arguments=[LaunchConfiguration('config_path')],
        remappings=[
            ('imu_propagate', '/vins_estimator/imu_propagate'),
            ('path', '/vins_estimator/path'),
            ('odometry', '/vins_estimator/odometry'),
            ('point_cloud', '/vins_estimator/point_cloud'),
            ('margin_cloud', '/vins_estimator/margin_cloud'),
            ('key_poses', '/vins_estimator/key_poses'),
            ('camera_pose', '/vins_estimator/camera_pose'),
            ('camera_pose_visual', '/vins_estimator/camera_pose_visual'),
            ('keyframe_pose', '/vins_estimator/keyframe_pose'),
            ('keyframe_point', '/vins_estimator/keyframe_point'),
            ('extrinsic', '/vins_estimator/extrinsic'),
            ('image_track', '/vins_estimator/image_track'),
        ],
    )

    mavros_bridge = Node(
        package='vins',
        executable='vins_to_mavros_odometry',
        name='vins_to_mavros_odometry',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_odometry_bridge')),
        parameters=[{
            'input_topic': LaunchConfiguration('bridge_input_topic'),
            'output_topic': LaunchConfiguration('bridge_output_topic'),
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'max_publish_rate_hz': ParameterValue(
                LaunchConfiguration('bridge_max_publish_rate_hz'),
                value_type=float),
            'reject_large_jumps': True,
            'max_position_jump_m': 2.0,
            'max_orientation_jump_deg': 45.0,
            'convert_linear_velocity_to_child_frame': True,
            'publish_mavros_static_tf': True,
        }],
    )

    vision_pose_bridge = Node(
        package='vins',
        executable='odometry_to_pose_stamped',
        name='vins_to_mavros_vision_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_vision_pose_bridge')),
        parameters=[{
            'input_topic': LaunchConfiguration('bridge_input_topic'),
            'output_topic': LaunchConfiguration('vision_pose_output_topic'),
            'frame_id': LaunchConfiguration('vision_pose_frame_id'),
            'max_publish_rate_hz': ParameterValue(
                LaunchConfiguration('vision_pose_rate_hz'),
                value_type=float),
        }],
    )

    local_position_path = Node(
        package='vins',
        executable='pose_to_path',
        name='mavros_local_position_path',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_local_position_path')),
        parameters=[{
            'input_topic': '/mavros/local_position/pose',
            'output_topic': '/mavros/local_position/path',
            'fixed_frame_id': 'world',
            'max_poses': 3000,
        }],
    )

    return LaunchDescription([
        config_path_arg,
        bridge_input_topic_arg,
        bridge_output_topic_arg,
        bridge_max_publish_rate_hz_arg,
        publish_odometry_bridge_arg,
        publish_vision_pose_bridge_arg,
        vision_pose_output_topic_arg,
        vision_pose_frame_id_arg,
        vision_pose_rate_hz_arg,
        publish_local_position_path_arg,
        vins_node,
        mavros_bridge,
        vision_pose_bridge,
        local_position_path,
    ])
