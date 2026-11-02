"""Lanza el detector de agujeros en la red (un único nodo Rust (r2r))."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    # Solo se sobrescriben los parámetros que se pasan como argumento
    overrides = {}
    for launch_arg, param in (('engine', 'inference.engine'), ('model_path', 'model.path'),
                              ('image_transport', 'image_transport')):
        if arg(launch_arg):
            overrides[param] = arg(launch_arg)
    if arg('is_rectified'):
        overrides['is_rectified'] = arg('is_rectified').lower() == 'true'

    return [Node(
        package='net_hole_detector_rust',
        executable='net_hole_detector_node',
        name='net_hole_detector',
        namespace=arg('namespace'),
        output='screen',
        parameters=[arg('params_file'), overrides,
                    {'use_sim_time': arg('use_sim_time').lower() == 'true'}],
        # r2r no puede resolver el remap de `image` para añadirle /compressed: se remapean los dos
        remappings=[('image', arg('image_topic')),
                    ('image/compressed', arg('image_topic') + '/compressed'),
                    ('camera_info', arg('camera_info_topic'))],
    )]


def generate_launch_description():
    share = get_package_share_directory('net_hole_detector_rust')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='net_hole_detector_rust'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'net_hole_detector.yaml')),
        DeclareLaunchArgument('image_topic',
                              default_value='/bluerov/stonefish/front_camera/image_color'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/bluerov/stonefish/front_camera/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('engine', default_value='',
                              description='tensorrt | cuda | cpu (vacío = el del YAML)'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='ONNX a cargar (vacío = el del YAML)'),
        DeclareLaunchArgument('image_transport', default_value='',
                              description='raw | compressed (vacío = el del YAML)'),
        DeclareLaunchArgument('is_rectified', default_value='',
                              description='true | false (vacío = el del YAML)'),
        OpaqueFunction(function=launch_setup),
    ])
