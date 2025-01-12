import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
################### user configure parameters for ros2 start ###################
xfer_format   = 4    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'


################### user configure parameters for ros2 end #####################



def generate_launch_description():

    config_file_path = os.path.join(
        get_package_share_directory('planner_manager'),
        'config',
        'planner_manager.yaml'
    )
    planner_manager_node = Node(
        package='planner_manager',
        executable='planner_manager_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_path]
    )
    icp_registration = Node(
        
        package='icp_registration',
        executable='icp_registration_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
                    "/home/z/Ouuu_fast_planner_2D/src/icp_registration/config/icp.yaml",
                    {'use_sim_time': True,
                    'pcd_path': "/home/z/Ouuu_fast_planner_2D/src/planner_manager/PCD/RMUC.pcd"}
                        ],
    )
    
    config = os.path.join(
        get_package_share_directory('fast_lio'), 'config', 'mid360.yaml')

    start_fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[config],
        output='screen',
        # on_exit=get_on_exit_action('fast_lio'),
    )

    config = os.path.join(
        get_package_share_directory('rm_serial_driver_only_speed'), 'config', 'serial_driver.yaml')
    rm_serial_driver_node = Node(
        package='rm_serial_driver_only_speed',
        executable='rm_serial_driver_only_speed_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    config = os.path.join(
        get_package_share_directory('linefit_ground_segmentation_ros'), 'launch', 'segmentation_params.yaml')

    linefit_ground_segmentation = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )
    pointcloud_to_laserscan = Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in',  ['/livox/lidar/pointcloud']),
            # remappings=[('cloud_in',  ['cloud_registered_body']),
            remappings=[('cloud_in',  ['/segmentation/obstacle']),            
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',
                'transform_tolerance': 0.01, 
                'min_height': -1.0,
                'max_height': 0.5,
                'angle_min': -3.14159, # -M_PI
                'angle_max': 3.14159, # M_PI
                'angle_increment': 0.0043,  # M_PI/360.0
                'scan_time': 0.33334,
                'range_min': 0.0,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    config_file_path = os.path.join(
            get_package_share_directory('livox_ros_driver2'),
            'config',
            'MID360_config.json'
        )
    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": config_file_path},
        {"cmdline_input_bd_code": cmdline_bd_code}
    ]
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )
    return LaunchDescription([livox_driver,planner_manager_node,start_fast_lio,linefit_ground_segmentation,pointcloud_to_laserscan,rm_serial_driver_node])
