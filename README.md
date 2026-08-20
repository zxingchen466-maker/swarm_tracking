单车轨迹跟踪仿真 (swarm_ws)

单台麦克纳姆轮小车的轨迹跟踪仿真：Gazebo 提供世界与车体，控制器仅通过话题通信（不依赖 Gazebo，可移植到真车）。环境：Ubuntu 20.04 + ROS Noetic + Gazebo 11 + catkin_tools。

【1. 编译与启动】
  cd ~/swarm_ws
  catkin build
  source devel/setup.bash                 （每个新终端都要执行）
  roslaunch mecanum_sim sim_control.launch （一键启动：世界 + 小车 + 轨迹生成 + 控制器）
  无 GPU 的虚拟机需先执行 export LIBGL_ALWAYS_SOFTWARE=1 再启动。

【2. 三个包】
  mecanum_sim    车模型 urdf/mecanum.xacro、世界 worlds/flat.world、启动文件 launch/
  consensus_ctrl 消息 msg/、参数 config/、节点 scripts/（轨迹生成 + 控制器）
  swarm_tools    录制 scripts/record.sh、画图 scripts/plot_tracking.py、bags/（已 gitignore）

【3. 话题】（命名空间 /robot_1）
  /robot_1/odom        nav_msgs/Odometry          里程计：小车实际位姿（控制器的反馈输入）
  /robot_1/ref         consensus_ctrl/TrajRef     参考轨迹：pos / vel / acc（位置、速度、加速度）
  /robot_1/cmd_vel     geometry_msgs/Twist        控制器下发的速度指令（限幅后）
  /robot_1/ctrl_debug  consensus_ctrl/CtrlDebug   调试：ex/ey/e_yaw 跟踪误差 + cmd_raw_* 限幅前指令

【4. 参数】（全部在 yaml，改完重启 launch 生效）
  consensus_ctrl/config/traj.yaml : traj_type: circle / fig8；t_ramp 平滑启动时长；circle.R/T、fig8.A/T 轨迹尺寸与周期；yaw_mode: fixed
  consensus_ctrl/config/ctrl.yaml : controller_type: pd；vmax/wmax 饱和限幅（1.0 m/s、2.0 rad/s）；timeout 0.5 s 超时保护；ctrl_params: Kp / Kd / Kp_yaw

【5. 录制与画图】
  另开一个终端，先 source devel/setup.bash，然后：
  bash ~/swarm_ws/src/swarm_tools/scripts/record.sh [轨迹] [控制器] [秒数]     例：record.sh fig8 pd 60
  python3 ~/swarm_ws/src/swarm_tools/scripts/plot_tracking.py <bag路径>        生成 3 张 PDF 到 bag 同目录
  PDF：1_trajectory.pdf（参考 vs 实际 + 放大镜 + 间隙标注）、2_tracking_error.pdf（误差曲线）、3_control_input.pdf（控制量 + 饱和线）
  验收指标：跟踪误差 < 0.10 m

【6. 更换控制器】（不动仿真与轨迹代码）
  步骤1：在 consensus_ctrl/scripts/controller_node.py 中继承 ControllerBase 编写新控制器类（只需实现 compute()）
  步骤2：把 ctrl.yaml 的 controller_type 改为新类名
