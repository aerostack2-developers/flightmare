from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os
import subprocess

AS2_WS_PATH = os.getenv('AEROSTACK2_WORKSPACE')
FLIGHTMARE_PATH = os.path.join(f"{AS2_WS_PATH}", "src/flightmare/flightrender/RPG_Flightmare/RPG_Flightmare.x86_64")

def kill_flightmare(_launch_context):
    process = subprocess.run("out=$(ps aux | grep flightros | grep -v 'grep' | awk -v col=2 '{print $col}'); printf 'Cleaning on exit.. ' ; for i in $out; do printf '[pid '$i'] ' && kill -9 $i; done; echo ''" , shell=True)

def generate_launch_description():
    scene_dict = {
        "INDUSTRIAL": '0',
        "WAREHOUSE": '1',
        "GARAGE": '2',
        "TUNELS": '3',
        "NATUREFOREST": '4',
        "PLANTA": '5'
    }

    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        DeclareLaunchArgument('render', default_value='true'),
        DeclareLaunchArgument('scene_id', default_value='1'),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('posx', default_value='0.0'),
        DeclareLaunchArgument('posy', default_value='0.0'),
        DeclareLaunchArgument('posz', default_value='0.0'),
        Node(
            package='flightros',
            namespace=LaunchConfiguration('drone_id'),
            executable='flight_pilot_node',
            name='flight_pilot',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"scene_id": LaunchConfiguration('scene_id'),
                "render": LaunchConfiguration('render'),
                "model": LaunchConfiguration('model'),
                "posx": LaunchConfiguration('posx'),
                "posy": LaunchConfiguration('posy'),
                "posz": LaunchConfiguration('posz')
                }]
        ),

        ExecuteProcess(
            cmd=[f'{FLIGHTMARE_PATH}'],
            log_cmd=True,
            sigterm_timeout='0',
            sigkill_timeout='0',
            on_exit=[
                OpaqueFunction(
                    function=kill_flightmare
                ),
                LogInfo(msg='Launching Flightmare..')
            ],
            condition=IfCondition(LaunchConfiguration("render"))
        )
    ])
