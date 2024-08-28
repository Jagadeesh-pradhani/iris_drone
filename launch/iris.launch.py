import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

import xacro

def generate_launch_description():

    world = os.path.join(get_package_share_directory('iris_drone'), 'worlds')
    launch_file_dir = os.path.join(get_package_share_directory('iris_drone'), 'launch')
    pkg_gazebo_tf_models = get_package_share_directory('iris_drone')
    model_name = 'iris_demo'

    world = os.path.join(
        get_package_share_directory('iris_drone'),
        'worlds',
        'odeh.world'
    )

    # sdf_file = os.path.join(
    #     get_package_share_directory('iris_drone'),
    #     'models',
    #     'iris_with_ardupilot/model.sdf'
    # )

    sdf_file = "/home/jagadeesh/iris/src/iris_drone/models/iris_demo/model.sdf"

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] \
            + ':' + pkg_gazebo_tf_models + '/models'
    else:
        model_path =  pkg_gazebo_tf_models + '/models'

    gazebo_ros = get_package_share_directory('gazebo_ros')
    
    gazebo_client = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
     )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )


    spawn_drone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', model_name,
            '-file', sdf_file
            # '-x', x_pose,
            # '-y', y_pose,
            # '-z', '0.01'
        ],
        output='screen',
    )

    # use_sim_time = LaunchConfiguration("use_sim_time", default="true")


    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'iris_publisher.launch.py')
        ),
    )



    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # namespace=model_ns,
        output='screen',
    )

    Odom_publisher = Node(
            package="iris_drone",
            executable="odom_pub",
            output="screen"
        )
    
    tf_publisher = Node(
            package="iris_drone",
            executable="tf_pub",
            output="screen"
        )
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_scan']
    )

    # Add odom publisher (using robot_localization's EKF)
    odom_publisher = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'base_link_frame': 'base_footprint',
            'world_frame': 'odom',
            'publish_tf': True
        }]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='drone_viz',
        # output='screen',
        #output={'both': 'log'},
        parameters=[{'queue_size': 100}],
        arguments=['-d', os.path.join(get_package_share_directory('iris_drone'), 'rviz', 'rviz.rviz')]
    )

    ardupilot = Node(
            package="iris_drone",
            executable="ardupilot",
            #namespace="drone",
            output="screen",
            prefix="xterm -e",
        )
    # print(use_sim_time)






    

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=world,
          description='SDF world file'),

        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),

        launch.actions.DeclareLaunchArgument(
            name='extra_gazebo_args',
            default_value='--verbose',
            description='Extra plugins for (Gazebo)'),

        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
          
        gazebo_server,
        gazebo_client,
        spawn_drone,
        tf_publisher,
        rviz,
        ardupilot,
        
    ])

    return ld


if __name__ == '__main__':#
    generate_launch_description()