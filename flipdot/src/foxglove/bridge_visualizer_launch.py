
import launch.actions
import launch_ros.actions
import third_party.foxglove_bridge.node_path
import third_party.foxglove_bridge.params


def generate_launch_description():
    return launch.LaunchDescription([

        launch.actions.SetEnvironmentVariable(name='ROS_DISTRO',
                                              value='humble'),

        # 1. Run the Foxglove Bridge
        launch_ros.actions.Node(
            executable=third_party.foxglove_bridge.node_path.NODE_PATH,
            output="screen",
            parameters=[
                third_party.foxglove_bridge.params.PARAMS_TO_DEFAULT_VALUES,
            ],
        ),
        # # 2. Run the visualizer to convert data to images
        # launch_ros.actions.Node(
        #     package="foxglove",
        #     executable="visualizer",
        #     output="screen",
        # ),
    ])