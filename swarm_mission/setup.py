from setuptools import setup

package_name = 'swarm_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrik Mohai',
    maintainer_email='patrik.mohai@gmail.com',
    description='Byzantine fault-tolerant drone swarm mission',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_navigator = swarm_mission.waypoint_navigator:main',
            'ground_truth_logger = swarm_mission.ground_truth_logger:main',
            'pointcloud_injector = swarm_mission.pointcloud_injector:main',
            'lidar_deskew_relay = swarm_mission.lidar_deskew_relay:main',
            'qos_poisoner = swarm_mission.qos_poisoner:main',
            'vision_pose_spoofer = swarm_mission.vision_pose_spoofer:main',
            'setpoint_hijacker = swarm_mission.setpoint_hijacker:main',
            'imu_injector = swarm_mission.imu_injector:main',
            'lidar_dos = swarm_mission.lidar_dos:main',
            'mavros_cmd_injector = swarm_mission.mavros_cmd_injector:main',
            'lidar_manipulator = swarm_mission.lidar_manipulator:main',
            'clock_filter = swarm_mission.clock_filter:main',
            'slam_failure_logger = swarm_mission.slam_failure_logger:main',
            'lidar_deskew_shim = swarm_mission.lidar_deskew_shim:main',
            'imu_injector_v2 = swarm_mission.imu_injector_v2:main',
            'network_attacker = swarm_mission.network_attacker:main',
        ],
    },
)
