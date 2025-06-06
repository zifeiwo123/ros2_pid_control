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
1. 启动驱动节点（发布target值）
ros2 run driver driver_node
2. 启动 PID 控制节点
ros2 run process process_node
3. 可视化跟踪效果
ros2 run rqt_plot rqt_plot
在 rqt_plot 界面中，添加以下title实现追踪：
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
    kp:       # 比例系数
    ki:       # 积分系数
    kd:       # 微分系数
    

实验结果
通过 rqt_plot 获取的跟踪曲线截图位于正文目录下，包含 X、Y、Z 三个轴的目标值与实际输出对比：
image_x.png
image_y.png
image_z.png

细节问题说明
代码文件首次编写成功在5月初，cmake里面寻找头文件路径是可行的，提交前再次运行发现有问题，找不到路径且生成hpp的路径出现路径重复，检索资料外加尝试怀疑是bug，暂时通过对编译器添加路径作为临时解决方法，不影响正常运行

贡献者
zifeiwo123 (540761267@qq.com)
许可证
本项目采用 MIT 许可证，详情见LICENSE文件。
