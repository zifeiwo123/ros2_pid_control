ROS 2 PID 控制

## 运行环境
- **操作系统**：Ubuntu 22.04.5 LTS
- **ROS 2版本**：Humble
- **Python版本**：Python 3.10.12
- **CMake版本**：3.22.1

代码获取与编译
1. 克隆仓库
bash
git clone https://github.com/zifeiwo123/ros2_pid_control.git
cd ros2_pid_control
2. 编译项目
bash
cd ~/catkin_ws
colcon build --packages-select process
source install/setup.bash
运行步骤
1. 启动驱动节点（发布目标值）
ros2 run driver driver_node
2. 启动 PID 控制节点
ros2 run process process_node
3. 可视化跟踪效果
ros2 run rqt_plot rqt_plot
在 rqt_plot 界面中，添加以下话题：
/target/target/x
/target/target/y
/target/target/z
/command/target/x
/command/target/y
/command/target/z

PID 参数调整说明
根据常见的工业场景使用经验，这里使用离散型增量式 PID 控制器，可根据实际需求调整：
process_node:
  ros__parameters:
    # PID参数
    kp: 0.5      # 比例系数
    ki: 0.0      # 积分系数
    kd: 0.01     # 微分系数
    
    # 输出限制 
    output_min: -2.0
    output_max: 2.0

实验结果
通过 rqt_plot 获取的跟踪曲线截图位于正文目录下，包含 X、Y、Z 三个轴的目标值与实际输出对比：
image_x.png
image_y.png
image_z.png

贡献者
zifeiwo123 (540761267@qq.com)
QingbiaoLI (qingbiao.qli@gmail.com)
许可证
本项目采用 MIT 许可证，详情见LICENSE文件。
