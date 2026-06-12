"""Launch three image_filtering nodes side by side for comparison.

All nodes subscribe to the SAME input topic and publish to distinct output
topics so the results of SimpleWB, GrayworldWB and CLAHE-on-LAB can be compared
(e.g. in rqt_image_view / RViz) at the same time.

Edit INPUT_TOPIC below (or remap it) to point at your camera/bag topic.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

INPUT_TOPIC = '/nautilus/front_camera/image_color'


def make_filter_node(name, filter_params):
    parameters = {
        'sub_topic': INPUT_TOPIC,
        'input_encoding': 'rgb8',
        'output_encoding': 'rgb8',
    }
    parameters.update(filter_params)
    return Node(
        package='image_filtering',
        executable='image_filtering_node',
        name=name,
        parameters=[parameters],
        output='screen',
    )


def generate_launch_description():
    simple_wb_node = make_filter_node(
        'simple_wb_node',
        {
            'pub_topic': '/filtered_image/simple_wb',
            'filter_params.filter_type': 'white_balancing',
            'filter_params.white_balancing.contrast_percentage': 10.0,
        },
    )

    gray_world_node = make_filter_node(
        'gray_world_node',
        {
            'pub_topic': '/filtered_image/gray_world',
            'filter_params.filter_type': 'gray_world',
            'filter_params.gray_world.max_gain': 4.0,
        },
    )

    clahe_node = make_filter_node(
        'clahe_node',
        {
            'pub_topic': '/filtered_image/clahe',
            'filter_params.filter_type': 'clahe',
            'filter_params.clahe.clip_limit': 2.0,
            'filter_params.clahe.tile_grid_size': 8,
        },
    )

    # Chained pipeline: white-balance first (Grayworld), then CLAHE on the
    # balanced frame. This node subscribes to the Grayworld node's OUTPUT
    # instead of the raw camera, so CLAHE enhances contrast on an already
    # de-cast image rather than amplifying noise on the raw green frame.
    gray_clahe_node = make_filter_node(
        'gray_clahe_node',
        {
            'sub_topic': '/filtered_image/gray_world',
            'pub_topic': '/filtered_image/gray_clahe',
            'filter_params.filter_type': 'clahe',
            'filter_params.clahe.clip_limit': 2.0,
            'filter_params.clahe.tile_grid_size': 8,
        },
    )

    # ---- Colour-restoration candidates (RGB in, RGB out) ----
    # Each reads the raw camera and publishes a restored RGB image, so you can
    # feed any of them into the segmentation model and compare.
    shades_of_gray_node = make_filter_node(
        'shades_of_gray_node',
        {
            'pub_topic': '/filtered_image/shades_of_gray',
            'filter_params.filter_type': 'shades_of_gray',
            'filter_params.shades_of_gray.norm_p': 6.0,
            'filter_params.shades_of_gray.max_gain': 4.0,
        },
    )

    white_patch_node = make_filter_node(
        'white_patch_node',
        {
            'pub_topic': '/filtered_image/white_patch',
            'filter_params.filter_type': 'white_patch',
            'filter_params.white_patch.percentile': 0.97,
            'filter_params.white_patch.max_gain': 4.0,
        },
    )

    underwater_compensation_node = make_filter_node(
        'underwater_compensation_node',
        {
            'pub_topic': '/filtered_image/underwater_compensation',
            'filter_params.filter_type': 'underwater_compensation',
            'filter_params.underwater_compensation.alpha': 1.0,
            'filter_params.underwater_compensation.compensate_blue': False,
            'filter_params.underwater_compensation.max_gain': 4.0,
        },
    )

    # Constant, scene-independent gains. Most temporally stable option - the
    # colour profile never shifts with scene content. Calibrate the gains once
    # for your water/lighting.
    fixed_gain_node = make_filter_node(
        'fixed_gain_node',
        {
            'pub_topic': '/filtered_image/fixed_gain',
            'filter_params.filter_type': 'fixed_gain',
            'filter_params.fixed_gain.gain_r': 1.8,
            'filter_params.fixed_gain.gain_g': 1.0,
            'filter_params.fixed_gain.gain_b': 1.2,
            'filter_params.fixed_gain.offset': 0.0,
        },
    )

    return LaunchDescription(
        [
            simple_wb_node,
            gray_world_node,
            clahe_node,
            gray_clahe_node,
            shades_of_gray_node,
            white_patch_node,
            underwater_compensation_node,
            fixed_gain_node,
        ]
    )
