import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('udp_camera_ros')
    default_params = os.path.join(pkg_share, 'config', 'camera_stream_params.yaml')

    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')
    autostart = LaunchConfiguration('autostart')

    camera_node = LifecycleNode(
        package='udp_camera_ros',
        executable='camera_node',
        name='camera_node',
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'port': ParameterValue(port, value_type=int)},
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(camera_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=camera_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter file for camera_node',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='5000',
            description='UDP listen port (must match the H.264 RTP sender)',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Auto configure+activate the lifecycle camera node',
        ),
        camera_node,
        configure_event,
        activate_on_inactive,
    ])
