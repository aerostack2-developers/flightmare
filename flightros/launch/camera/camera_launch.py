import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    drone_id = DeclareLaunchArgument('drone_id', default_value='drone0')
    debug = DeclareLaunchArgument('debug', default_value='0')
    use_unity_editor = 'false'
    # RPG Flightmare Unity Render.
    rpg_flightmare_render = Node(package='flightrender',
                                 executable='RPG_Flightmare.x86_64',
                                 name='rpg_flightmare_render',
                                 #  namespace=LaunchConfiguration('drone_id'),
                                 output='screen',
                                 emulate_tty=True
                                 )
    # if debug:
    #   # camera_node(launch-prefix="gdb -ex run --args")
    # else:
    camera_node = Node(package='flightros',
                       executable='camera_node',
                       name='camera',
                       namespace=LaunchConfiguration('drone_id'),
                       output='screen',
                       emulate_tty=True
                       )

    # rviz = Node(package='rviz',
    #                    executable='rviz',
    #                    name='rviz',
    #                    # args="-d $(find flightros)/launch/camera/camera.rviz"
    #                    output='screen',
    #                    emulate_tty=True
    #                    )

    return launch.LaunchDescription([drone_id,
                                     rpg_flightmare_render,
                                    #  camera_node,
                                     ])
