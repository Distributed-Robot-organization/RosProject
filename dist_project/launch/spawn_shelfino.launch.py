import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler, OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess, TimerAction
from launch import LaunchDescription, LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch.events.process import ProcessExited
from launch.event_handlers import OnProcessExit

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import logging

import yaml, re

def start_shelfini(configs, shelfino_desc_pkg, shelfino_nav2_pkg):

    nodes = []
    shelfini_names = []
    print("TRYNG TO START")

    # use_sim_time = True if context.launch_configurations['use_sim_time'] == 'true' else False

    with open (configs["map_config"], 'r') as f:
        shelfino_config_yaml = yaml.load(f, Loader=yaml.FullLoader)
        shelfino_config_yaml_ros = shelfino_config_yaml["/**"]["ros__parameters"]
        print(shelfino_config_yaml)
        for shelfino in range(len(shelfino_config_yaml_ros['init_names'])):
            if shelfino_config_yaml_ros['init_rand'][shelfino]:
                raise Exception(f"Shelfino {shelfino} is set to random pose, but should have been taken care of by the map generator")

            shelfino_name = shelfino_config_yaml_ros['init_names'][shelfino]
            if "evader" in shelfino_name or "pursuer" in shelfino_name:
                continue
            shelfini_names.append(shelfino_name)
            shelfino_pose_x = shelfino_config_yaml_ros['init_x'][shelfino]
            shelfino_pose_y = shelfino_config_yaml_ros['init_y'][shelfino]
            shelfino_pose_yaw = shelfino_config_yaml_ros['init_yaw'][shelfino]
            max_camera_depth = shelfino_config_yaml["/**"]["ros__parameters"]["shelfino_additions"]["max_camera_depth"]

            print(f"Spawning {shelfino_name} at ({shelfino_pose_x}, {shelfino_pose_y}, {shelfino_pose_yaw})")

            rsp_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(configs["project_package"], 'launch', 'rsp_test.launch.py')
                ]),
                launch_arguments= {
                    'use_sim_time': "true",
                    'shelfino_name': shelfino_name,
                    "max_camera_depth" :str(max_camera_depth)
                }.items()
            )

            nav2_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(shelfino_nav2_pkg, 'launch', 'shelfino_nav.launch.py')]
                ),
                launch_arguments= {
                    'use_sim_time': configs['use_sim_time'],
                    'robot_name': shelfino_name,
                    'map_file' : configs['map_file'],
                    'nav2_params_file' : configs['nav2_params_file'],
                    'rviz_config_file': configs['nav2_rviz_config_file'],
                    'initial_x': str(shelfino_pose_x),
                    'initial_y': str(shelfino_pose_y),
                    'initial_yaw': str(shelfino_pose_yaw),
                    'set_initial_pose': 'true',
                    'headless' : 'true',
                }.items()
            )


            spawn_shelfino_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                        '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                        '-entity', shelfino_name,
                        '-robot_namespace', shelfino_name,
                        '-x', PythonExpression(["'", str(shelfino_pose_x), "'"]),
                        '-y', PythonExpression(["'", str(shelfino_pose_y), "'"]),
                        '-z', '0.0',
                        '-Y', PythonExpression(["'", str(shelfino_pose_yaw), "'"]),
                ]
            )

            destroy_shelfino_node = Node(
                package='shelfino_gazebo',
                executable='destroy_shelfino',
                name='destroy_shelfino',
                output='screen',
                namespace=shelfino_name,
            )

            nodes += [
                rsp_launch_file,
                spawn_shelfino_node,

                nav2_launch_file,
                destroy_shelfino_node
            ]
            #nodes += depth_perception_corrections_nodes

    return nodes , shelfini_names # + evaluate_rviz(configs,shelfini_names)



# def evaluate_rviz(configs, shelfini_names):
#     """
#     This function allows for launching just one Rviz instance for all the robots.
#     It takes the rviz config file and creates a new one with the correct items
#     multiplied for all the robots.
#     :param context: The context of the launch including the launch config.
#     """
#     print("Passed shelfini_names", shelfini_names)
#     shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')

#     rviz_path = configs['nav2_rviz_config_file']
#     cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', f"shelfini_{len(shelfini_names)}_nav.rviz")

#     output_config = {}
#     with open(rviz_path, 'r') as f_in:
#         rviz_config= yaml.load(f_in, Loader=yaml.FullLoader)
#         for key in rviz_config.keys():
#             if key != 'Visualization Manager':
#                 output_config[key] = rviz_config[key]

#         # Add everything that is not displays or tools
#         output_config['Visualization Manager'] = {}
#         for key in rviz_config['Visualization Manager'].keys():
#             if key != 'Displays' and key != 'Tools':
#                 output_config['Visualization Manager'][key] = rviz_config['Visualization Manager'][key]

#         # Configure displays for Rviz
#         displays = rviz_config['Visualization Manager']['Displays']
#         output_config['Visualization Manager']['Displays'] = []
#         for display in displays:
#             if type(display) is not dict:
#                 raise Exception("[{}] Display `{}` is not a dictionary".format(__file__, display))
#             if "shelfinoX" in str(display):
#                 display_str = str(display)
#                 for shelfino_name in shelfini_names:
#                     output_config['Visualization Manager']['Displays'].append(
#                         yaml.load(display_str.replace("shelfinoX", shelfino_name), Loader=yaml.FullLoader))
#             else:
#                 output_config['Visualization Manager']['Displays'].append(display)

#         # Configure tools for Rviz
#         tools = rviz_config['Visualization Manager']['Tools']
#         output_config['Visualization Manager']['Tools'] = []
#         for tool in tools:
#             if type(tool) is not dict:
#                 raise Exception("[{}] Tool `{}` is not a dictionary".format(__file__, tool))
#             if "shelfinoX" in str(tool):
#                 tool_str = str(tool)
#                 for shelfino_name in shelfini_names:
#                     output_config['Visualization Manager']['Tools'].append(
#                         yaml.load(tool_str.replace("shelfinoX", shelfino_name), Loader=yaml.FullLoader))
#             else:
#                 output_config['Visualization Manager']['Tools'].append(tool)

#         print(output_config)
#         print("Writing to", cr_path)

#         with open(cr_path, 'w+') as f_out:
#             yaml.dump(output_config, f_out, default_flow_style=False)

#     configs['rviz_config_file'] = cr_path

    # return [Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', configs['rviz_config_file']],
    #     parameters=[
    #         {'use_sim_time': True if configs['use_sim_time'] == 'true' else False}
    #     ],
    # )]

def define_yaml_templates(configs):



    nav2_yaml_file = configs["nav2_params_file_path_template"]
    map_yaml_file = configs["map_file"]

    # Load template
    with open(nav2_yaml_file, 'r') as f:
        config = yaml.safe_load(f)

    # Substitute value
    config['map_server']['ros__parameters']['yaml_filename'] = map_yaml_file

    # Write temporary file
    tmp_path = '/tmp/temp_nav2_config.yaml'
    with open(tmp_path, 'w') as f:
        yaml.dump(config, f)

    configs["nav2_params_file_path"]  = tmp_path
    return configs


def generate_launch_description():

    shelfino_desc_pkg  = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg  = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg  = get_package_share_directory('shelfino_gazebo')
    dist_project_pkg        = get_package_share_directory('dist_project')
    configs = {}


    configs["nav2_params_file_path_template"] = os.path.join(dist_project_pkg, 'config', 'shelfino_nav.yaml')
    configs["map_config"] = os.path.join(dist_project_pkg, 'config', 'shelfino_params.yaml')
    configs["project_package"] = dist_project_pkg

    # General arguments
    configs["use_sim_time"] = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo simulation arguments
    configs["use_gui"]           = LaunchConfiguration('use_gui', default='true')
    configs["use_rviz"]          = LaunchConfiguration('use_rviz', default='true')
    map_name = "NOT DEFINED"
    with open (configs["map_config"], 'r') as f:
        shelfino_config_yaml = yaml.load(f, Loader=yaml.FullLoader)
        shelfino_config_yaml = shelfino_config_yaml["/**"]["ros__parameters"]
        map_name = shelfino_config_yaml["map"]

    configs["map_file"]              = os.path.join(dist_project_pkg, 'worlds', f'{map_name}/{map_name}.yaml')

    configs = define_yaml_templates(configs)

    #Defined config files
    configs["rviz_config_file"]  = LaunchConfiguration('rviz_config_file', default=os.path.join(dist_project_pkg, 'config', 'overall_map.rviz'))
    configs["gazebo_world_file"] = LaunchConfiguration('gazebo_world_file', default=os.path.join(dist_project_pkg, 'worlds', f'{map_name}/{map_name}.world'))

    # Navigation arguments
    configs["nav2_params_file"]      = LaunchConfiguration('nav2_params_file', default=configs["nav2_params_file_path"])
    configs["nav2_rviz_config_file"] = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))


    nodes_to_launch = []
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    #
    nodes_to_launch, shelfini_names = start_shelfini(configs, shelfino_desc_pkg, shelfino_nav2_pkg)
    #nodes_to_launch+=evaluate_rviz(configs, shelfini_names)


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_gaze_pkg, 'launch', 'shelfino.launch.py')]
        ),
        launch_arguments= {
            'use_sim_time': configs["use_sim_time"],
            'use_gui': configs["use_gui"],
            'use_rviz':"false",
            'gazebo_world_file': configs["gazebo_world_file"],
            'spawn_shelfino': 'false',
        }.items()
    )

    """ if len(shelfini_names)==2:
        nodes_to_launch+=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', configs['rviz_config_file']],
                parameters=[
                    {'use_sim_time': "true" if configs['use_sim_time'] == 'true' else "false"}
                ],
            )
        ] """
    return LaunchDescription(nodes_to_launch+[gazebo_launch])
