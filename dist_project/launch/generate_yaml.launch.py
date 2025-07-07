from launch.actions import OpaqueFunction

def substitute_param_file(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    template_path = os.path.join(
        get_package_share_directory('your_package_name'),
        'params', 'nav2_params_template.yaml'
    )
    output_path = f'/tmp/nav2_params_{robot_name}.yaml'

    with open(template_path, 'r') as f:
        content = f.read()

    with open(output_path, 'w') as f:
        f.write(content.replace('ROBOT_NAME', robot_name))

    return [Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        namespace=robot_name,
        parameters=[output_path],
        output='screen'
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='shelfino1'),
        OpaqueFunction(function=substitute_param_file)
    ])
