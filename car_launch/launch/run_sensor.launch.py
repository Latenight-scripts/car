import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # =========================================================
    # 在这行修改 IMU 串口号
    MY_IMU_PORT = '/dev/ttyUSB0' 
    MY_PASSWORD = '1'
    # =========================================================

    lidar_share = get_package_share_directory('lidar')
    imu_share = get_package_share_directory('imu')
    rs_to_velo_share = get_package_share_directory('rs_to_velodyne')


    # 2. 定义启动项
    # [1/4] Lidar (立即启动)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'start.launch.py')
        )
    )


    # [2/4] IMU (fdilink_ahrs 驱动)
    # 串口/波特率在 fdilink_ahrs/launch/ahrs_driver.launch.py 内配置，默认 /dev/ttyUSB0 921600
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_share, 'launch', 'ahrs_driver.launch.py')
        )
    )

    # [3/4] RS_to_Velodyne (点云转换)
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_to_velo_share, 'launch', 'rs_to_velodyne.launch.py')
        )
    )

    # 3. 设置延时启动逻辑 (单位：秒)
    # 延时 2 秒启动 IMU (防止 USB 抢占，并给权限修改留时间)
    delayed_imu = TimerAction(
        period=2.0,
        actions=[imu_launch]
    )

    # 延时 3 秒启动转换节点 (确保雷达数据已就绪)
    delayed_conversion = TimerAction(
        period=3.0,
        actions=[rs_launch]
    )



    return LaunchDescription([
        lidar_launch,
        delayed_imu,
        delayed_conversion,
        # delayed_camera
        # delayed_rosbridge
    ])