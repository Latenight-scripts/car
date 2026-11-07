# 修复启动后规划节点退出

实车终端显示，bridge_node在接收第一个目标前调用_publish_diag，读取不存在的core._goal，引发AttributeError并退出。定位与跟踪节点继续运行，但规划链路已断，跟踪发布零速度。

旧VehicleBridgeCore.step会初始化目标属性，新RealtimeVehicleBridgeCore覆盖了该方法，却没有在构造函数中初始化同一属性。本次将未收到目标表示为None；未收到目标时控制器仍返回no_goal并停车。收到合法目标后才进入规划。诊断新增goal_received，未定义目标与非有限诊断数值以标准JSON null发布。

另修复car_ctr中enable=false发送零速度后仍可能落入非零Run分支的问题：禁用消息发出零速度后立即返回；enable=true且速度为零仍允许原有机械模式切换。

运行源码只更新两个包的三个文件：

- rda_dwa_real/rda_dwa_real/realtime_vehicle_core.py
- rda_dwa_real/rda_dwa_real/bridge_node.py
- control/car_ctr/src/car_ctr.cpp

验证通过60项Python回归，包括实际节点的控制周期、启动无目标、先定位后目标、原点/5米前方/近侧向目标、感知过期停车和独立spawn异步规划进程。实际car_ctr.cpp通过C++17与ROS/UDP内存替身编译运行，验证禁用消息在已可运动及模式切换期间只发零速度。未运行ROS中间件或实车，修复后的车机效果需要重新测试。

车机更新时，先停止本次launch并保存记录。在工作空间外拉取仓库，备份并覆盖上述三个文件，或替换rda_dwa_real与control/car_ctr两个完整包。然后在现有ROS工作空间根目录重新构建：

```bash
colcon build --symlink-install --packages-up-to rda_dwa_real car_ctr --cmake-clean-cache
source install/setup.bash
ros2 launch car_launch run_rda_dwa.launch.py
```

终端还显示车机系统时间为1970年，大量源文件与ROS库被报为未来时间。请在停止车辆及ROS进程后校正系统时间，再构建/启动；底层TTimer使用gettimeofday，不能在车辆运行期间大幅跳变系统时间。联网并配置时间同步时可启用NTP；没有网络时按真实当地时间手动校时。以date和timedatectl status确认结果，不能把NTP开关打开当作同步成功。

曾出现从car复制到car_gpt后仍引用旧路径的CMakeCache错误。--cmake-clean-cache会重新生成所构建CMake包的缓存；若仍报旧路径，需要在停止运行后备份并重新生成相应构建/安装目录，保留未在本交付仓库中的原有驱动及地图环境。

重新启动后另开同环境终端检查：

```bash
ros2 node list
ros2 topic echo /rda/diag --once
ros2 topic hz /rda/plan
```

节点列表应持续包含/rda_dwa_real_bridge。发布目标前diag应为reason=no_goal、goal=null；收到目标后goal_received=true，后续reason可能是motion、planning_pending、自转请求或明确的输入/安全等待原因。无新报错仅证明启动缺陷修复，不能证明所有实车场景已经通过。

本次原始记录目录在旧car工作空间的record_data中；记录节点启动的日志能确认路径，实际JSON与rosbag是否完整仍需检查recorder_health、writer_summary及raw_bag元数据。
