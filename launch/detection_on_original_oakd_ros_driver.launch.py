import os
import yaml
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    pkgdir = get_package_share_directory('tiny_gestures')
    pkgParamsDir = os.path.join(pkgdir, 'params')
    ### DEPTHAI default ros driver (has problems with inference, as the spatial coordinates are very very bad)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    ########################################################################
    oak_model = "OAK-D-W"
    nnName = "gesture_yolov4tiny_openvino_2022.1_6shave.blob"
    nnresourceBaseFolder = os.path.join(pkgdir, "models")
    nnblobpath = os.path.abspath(os.path.join(nnresourceBaseFolder, nnName))
    depthai_config_file = os.path.join(pkgParamsDir, 'oakd.yaml')
    nnDepthaiConfig = "gestures_tinyyolo4_description_depthai.json"
    nnDepthaiConfigPath = os.path.abspath(os.path.join(pkgParamsDir, nnDepthaiConfig))
    ### HACK: take  json nn config file and replace 
    ### model:
    ###     model_name: <path>
    ### in it, because i dont want to hardcode the path for the model blob into the json config file
    with open(nnDepthaiConfigPath, 'r') as file:
        j = json.load(file)
    j["model"]["model_name"] = nnblobpath
    newDepthAiNnConfig = "/tmp/yolo4_gestures_tmp.json"
    with open(newDepthAiNnConfig, 'w') as file:
        json.dump(j, file, indent=4)
    ### HACK: take config file and add 
    ### nn:
    ###     i_nn_config_path: $HOME
    ### into it, because i dont want to hardcode the path for the model json into the yaml file
    ### and ros2 still has no env resolver in its yamls
    ### i dont want to set the ros parameter in the node construction, as the launchfile from dephai is used
    ### and i dont want to have a semi redundant copy of it only for nn paths
    with open(depthai_config_file, 'r') as file:
        print(depthai_config_file)
        d = yaml.safe_load(file)
    ### use 'newDepthAiNnConfig' as the configuration file instead of 'nnDepthaiConfigPat'
    d["/oak"]["ros__parameters"]["nn"]["i_nn_config_path"] = newDepthAiNnConfig
    newDepthAiConfig = "/tmp/depthai_params_tmp.yaml"
    with open(newDepthAiConfig, 'w') as file:
        yaml.dump(d, file, indent=4)
    depthai_config_file = yaml.dump(d)
    ###
    ########
    ###
    ### DEPTHAI default ros driver (has problems with inference, as the spatial coordinates are very very bad)
    camera_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": "oak",
                              "camera_model": oak_model,
                              "params_file": newDepthAiConfig,
                              "parent_frame": "oak-d-base-frame",
                               "cam_pos_x": "0.0",
                               "cam_pos_y": "0.0",
                               "cam_pos_z": "0.0",
                               "cam_roll": "0.0",
                               "cam_pitch": "0.0",
                               "cam_yaw": "0.0",
                               "use_rviz": "False",
                               }.items()
        )
    gesture_tracker_node = launch_ros.actions.Node(
            package='tiny_gestures',
            executable='gesture_detection_inference_on_oakd',
            name='gesture_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[{
                ### default rgb / depth (rgb should be 416x416)
                # 'rgb_topic': '/oak/rgb/image_raw',
                # 'depth_topic': '/oak/stereo/image_raw',
                ### passthrough rgb fro nn pipeline (possibly resized)
                'rgb_topic': '/oak/nn/passthrough/image_raw',
                'depth_topic': '/oak/nn/passthrough_depth/image_raw',
                'nn_topic': '/oak/nn/spatial_detections',
                "visualize": True,
            }]
    )

    oak_z = 1.0
    trafoParams = ["--x", "0.00",   "--y", "-0.0",   "--z", "%s"%oak_z,     "--roll","-0.0" ,   "--pitch", "0.0" , "--yaw", "0.0", "--frame-id", "base_link", "--child-frame-id", "oak-d-base-frame"]
    base_link_trafo = launch_ros.actions.Node(package='tf2_ros', executable='static_transform_publisher', name='trafo1', arguments = trafoParams)

    # rviz_node = launch_ros.actions.Node(
    #         package='rviz2', executable='rviz2', output='screen',
    #         arguments=['--display-config', default_rviz])

    ld = LaunchDescription()
    ld.add_action(camera_nodes)
    ld.add_action(gesture_tracker_node)
    ld.add_action(base_link_trafo)

    return ld

