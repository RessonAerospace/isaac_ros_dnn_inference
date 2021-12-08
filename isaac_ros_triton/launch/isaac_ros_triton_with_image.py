# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Load and launch mobilenetv2-1.0 onnx model through Triton node."""
    launch_dir_path = os.path.dirname(os.path.realpath(__file__))
    model_dir_path = launch_dir_path + "/../../models"

    # The Image to Tensor encoder
    encoder_node = ComposableNode(
        name="dnn_image_encoder",
        package="isaac_ros_dnn_encoders",
        plugin="isaac_ros::dnn_inference::DnnImageEncoderNode",
        parameters=[
            {
                "network_image_width": 960,
                "network_image_height": 544,
                "network_image_encoding": "rgb8",
                "network_normalization_type": "positive_negative",
                "tensor_name": "input_tensor",
            }
        ],
        remappings=[("encoded_tensor", "tensor_pub")],
    )

    # The Triton inference server node
    triton_node = ComposableNode(
        name="triton_node",
        package="isaac_ros_triton",
        plugin="isaac_ros::dnn_inference::TritonNode",
        parameters=[
            {
                "model_name": "peoplesemsegnet",
                "model_repository_paths": [model_dir_path],
                "max_batch_size": 0,
                "input_tensor_names": ["input_tensor"],
                "input_binding_names": ["input_1"],
                "output_tensor_names": ["output_tensor"],
                "output_binding_names": ["softmax_1"],
            }
        ],
    )

    triton_container = ComposableNodeContainer(
        name="triton_container",
        namespace="triton",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[encoder_node, triton_node],
        output="screen",
    )

    return launch.LaunchDescription([triton_container])
