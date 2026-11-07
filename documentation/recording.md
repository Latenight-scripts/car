# 实车诊断记录升级

已修改 diag_recorder.launch.py 引用的记录节点，以及感知、规划、跟踪、底盘的诊断发布。当前源码包含前面两项算法工作和本次日志升级。记录节点只读订阅/参数服务并写文件；新 UDP 旁路只记录报文与发送结果，不改变合法运动指令的协议。

## 需要这次升级的原因

之前的 snapshots.jsonl 是各话题最新缓存拼接的周期快照，不能视为同步数据；轨迹与障碍摘要无法还原完整规划/感知过程。/car_state 里的四轮默认零值也不证明收到了四轮实测速度。UDP 接收线程每次覆盖 rec_buf，驱动在20Hz只读到了最后一包，因此ROS反馈率不能代表所有UDP回包率。

现在逐条保存完整小消息，记录源时间、ROS接收时间、单调接收时间、墙钟、全局序号和话题序号，另保存原始点云/IMU/TF rosbag。主快照改为20Hz，/car_cmd、控制诊断等仍按实际收到的约50Hz逐条保留。快照仍明确标为非同步缓存，分析时要按消息和生产者时间对齐。

## 默认记录内容

| 环节 | 话题/数据 | 主要用途 |
| --- | --- | --- |
| 定位与坐标 | /localization 全Odometry、/imu/data、bag中的/tf与/tf_static | 位姿、协方差、原始IMU、定位源时间和坐标变换 |
| 感知 | /rslidar_points 原始bag、/rda/obstacles 全对象、/rda/perception_diag | 过滤/聚类/KF耗时、丢帧原因、实际使用定位、KF前检测框及输出轨迹 |
| 规划 | /rda/plan 全参考、/rda/diag、/cmd_vel、legacy轨迹 | 采样状态、规划耗时、结果年龄、丢弃原因、选定速度/曲率、模式及恢复子目标 |
| 跟踪与控制 | /rda/control_diag、/car_cmd、/rda/session_state | 投影位置、横向/航向误差、曲率、PID积分、加速度/转向率状态、执行净空、自转阶段和结果 |
| 底层 | /car_driver/diag、/car_driver/io、/car_state | 命令看门狗、锁存模式、机械等待、真实调用发送的内容、socket发送返回值及errno、socket原始回包、每轮反馈年龄 |
| 配置与版本 | runtime_parameters.jsonl、/parameter_events、provenance.jsonl、topic_graph.jsonl | 当前节点实际ROS参数、运行期间修改、源码/二进制SHA256、发布者消息类型与QoS |
| 实验标记 | experiment_label、/rda/test_note | 操作者干预、急停、障碍类别、视频对照等说明 |
| 记录质量 | recorder_health.jsonl、writer_summary.json、bag_console.log | 队列积压、丢记录、磁盘空间、bag状态、是否正常退出排空 |

控制诊断里的 e_th、delta_ff、correction 使用rad；旧 ctrl_status 的显示值使用deg。/car_cmd.angle为deg，ACK速度为m/s，自转速度为rad/s。实际舵角目前没有明确的测量字段，日志标记 unavailable，不把命令角或默认零当成实测角；Wheel报文的所有原始列仍可在UDP原文中确认。

底层IO分为：tx_send_called（驱动调用发送）、tx_socket_result（sendto返回值，不等于底盘已执行）、rx_socket（接收线程看到的原始包）、rx_observed（20Hz驱动读取缓存的样本）。socket旁路最多缓存256包，每包最多1024字节，每驱动周期取64包；溢出保留新包并累计 socket_diagnostic_dropped。正常协议包的传输参数保持原值；另补了畸形Wheel消息长度检查、接收缓冲终止符及有界拷贝，避免记录/解析越界。

## 文件与容量

每次启动生成独立 run_... 目录，包含：

- messages_000001.jsonl 等：所有已订阅小消息的完整到达记录，默认64MiB分卷。
- snapshots.jsonl / events.jsonl：概览快照和事件；plans.jsonl / sessions.jsonl：方便直接读取的原子参考和会话。
- runtime_parameters.jsonl：真正的只读远程参数服务返回及参数事件。params.json 仅为录制配置与指定YAML/默认值副本，已明确区分二者。
- provenance.jsonl / topic_graph.jsonl：版本、代码文件与实际发布者/QoS信息。
- recorder_health.jsonl / writer_summary.json：录制质量与退出时排空情况。
- raw_bag/：原始点云、IMU、TF与关键消息的rosbag2数据和metadata.yaml。
- bag_qos.yaml / bag_console.log：实际bag订阅QoS与启动、录制错误信息。

默认启用raw bag，单分卷目标512MiB、总容量约8GiB，并保留2GiB磁盘空间。总量与空闲空间每5s检查，可能有少量超出；达到限制只停止bag，JSON诊断继续记录。JSON队列默认4096条/32MiB，关键事件另有256条/1MiB预留；磁盘不足512MiB时会明确记录/报告写入失败，不默默假装采集成功。

不同ROS版本的bag命令选项可能不同：代码先检查本机 ros2 bag record --help，再选择支持的分卷/缓存参数。缺少rosbag、QoS选项、存储插件或空间时，bag会显示unavailable/failed及原因，JSON记录仍继续。原始点云必须依赖这份bag，单独JSON不能替代点云复盘。

## 车机启动与实验

当前电脑没有ROS，本次未运行车机、DDS或UDP。将完整新版本放入车机ROS工作空间后重新构建，尤其common头文件、car_ctr和rda_dwa_real；若从旧算法版本升级，还需重新构建rda_dwa_real_msgs。

```bash
# 在车机ROS工作空间根目录执行
colcon build --packages-up-to common control_msgs rda_dwa_real_msgs rda_dwa car_ctr rda_dwa_real car_launch
source install/setup.bash

# 原来的一键启动；默认diag=true、record_bag=true，目标仍由你发给小车
ros2 launch car_launch run_rda_dwa.launch.py experiment_label:="static_6m"

# 可以指定SSD目录和bag总容量，避免写入慢速SD卡
ros2 launch car_launch run_rda_dwa.launch.py diag_out_dir:=/path/to/ssd/record_data bag_max_gb:=8.0 experiment_label:="pedestrian_crossing"

# 独立记录（算法另行启动；不会启动车辆控制节点）
ros2 launch rda_dwa_real diag_recorder.launch.py out_dir:=/path/to/record_data

# 只记录小消息，不录原始点云bag
ros2 launch rda_dwa_real diag_recorder.launch.py record_bag:=false

# 如需定位侧转换点云或额外实测传感器话题，可添加；需要话题实际存在
ros2 launch rda_dwa_real diag_recorder.launch.py bag_extra_topics:="/velodyne_points,/measured_steering"

# 可在发生人为干预时留一个非控制注记
ros2 topic pub --once /rda/test_note std_msgs/msg/String "{data: '按下急停，行人进入车前'}"
```

正常按Ctrl+C退出后，等writer_summary.json及raw_bag/metadata.yaml生成。后续请把**整次run目录**打包，包括raw_bag目录里的metadata与所有db3/mcap分卷、JSON和bag_console.log。只发snapshots无法分析原始感知与完整时序。若bag失败或达到上限，仍保留全部健康信息，不删错误日志。

可用本包的inspect_recording.py在无ROS电脑上快速检查逐消息计数、时间倒序、序号缺口、写盘错误及bag元数据是否存在：

```bash
python docs/implementation/20261107_recorder/inspect_recording.py /path/to/run_directory
```

## 已验证与实际限制

42项Python回归通过，包含之前算法的32项，以及消息完整性、源时间倒序、队列/磁盘边界、分卷排空、bag命令/QoS/配额替身、实际诊断节点回调和反馈有效性。实际car_ctr.cpp用ROS/UDP内存替身经C++17编译运行；实际myudp.hpp也已编译，旁路以20000次双线程入队验证容量、顺序和覆盖计数，全程没有创建网络套接字。

1992条真实Python记录回调的离线突发注入中，回调p95约0.07ms、p99约0.08ms，2134条接受记录全部写出，无丢记录或写盘错误。这是本机替身基准，不能代表车机DDS、实际点云bag和SD卡吞吐。实际开销会出现在模块周期、recorder_health和队列积压中。

旁路和JSON自身均有容量边界，best-effort订阅也可能丢DDS消息；多个序号和丢弃计数用于识别这些问题，不声称绝对不丢包。原UDP缓存的控制读取方式仍保留，raw socket旁路用于看清这个链路，而不是重写底盘协议。没有对KF/导航/控制速度参数重新调参。

完整包 deployment_20261107_with_logging.zip 包含前面算法改进与本次日志升级。logging_changes.patch是相对上一版的单独日志补丁；rollback_logging.py撤销本次日志变更并保留之前算法版本，rollback.py可恢复到两项算法工作之前的原始源码。所有脚本先检查SHA256，默认只检查，加--apply才修改，遇到后续编辑会拒绝覆盖。
