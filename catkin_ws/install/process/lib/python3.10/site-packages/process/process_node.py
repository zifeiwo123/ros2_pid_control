import rclpy
from rclpy.node import Node
from driver_msgs.msg import Target
from collections import deque
import math

class DebugPIDController:
    """带调试信息和安全保护的PID控制器"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-10.0, 10.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0
        self.output_min, self.output_max = output_limits
        self.debug_info = {}  # 存储调试信息

    def update(self, setpoint, process_value, dt):
        """执行PID计算并返回控制输出"""
        # 安全检查：确保dt有效
        if dt <= 0:
            self.debug_info = {
                'dt': dt,
                'error': 0.0,
                'p_term': 0.0,
                'i_term': 0.0,
                'd_term': 0.0,
                'output': 0.0,
                'warning': 'Invalid dt (dt <= 0)'
            }
            return 0.0

        # 计算误差
        error = setpoint - process_value
        
        # 计算PID各项
        p_term = self.kp * error
        
        # 积分项计算与限制
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # 微分项计算
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # 计算总输出
        output = p_term + i_term + d_term
        
        # 输出限制（防止饱和）
        if output > self.output_max:
            output = self.output_max
            self.debug_info['warning'] = 'Output saturated (too large)'
        elif output < self.output_min:
            output = self.output_min
            self.debug_info['warning'] = 'Output saturated (too small)'
        
        # 存储调试信息
        self.debug_info = {
            'dt': dt,
            'setpoint': setpoint,
            'process_value': process_value,
            'error': error,
            'p_term': p_term,
            'i_term': i_term,
            'd_term': d_term,
            'output': output,
            'integral': self.integral
        }
        
        # 更新上一次误差
        self.last_error = error
        
        return output

    def get_debug_info(self):
        """获取调试信息"""
        return self.debug_info

class ProcessNode(Node):
    def __init__(self):
        super().__init__('process_node')
        
        # 创建订阅者，订阅target话题
        self.subscription = self.create_subscription(
            Target,
            'target',
            self.target_callback,
            10
        )
        self.subscription  # 防止未使用变量警告
        
        # 创建发布者，发布command话题
        self.publisher_ = self.create_publisher(Target, 'command', 10)
        
        # 创建定时器（100Hz，周期0.01秒）
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 消息窗口和最新消息存储
        self.message_window = deque(maxlen=5)
        self.latest_message = None
        
        # PID控制器初始化（参数可根据需求调整）
        self.pid_x = DebugPIDController(kp=10, ki=0.0, kd=0.01, output_limits=(-2.0, 2.0))
        self.pid_y = DebugPIDController(kp=10, ki=0.0, kd=0.01, output_limits=(-2.0, 2.0))
        self.pid_z = DebugPIDController(kp=12.5, ki=0.0, kd=0.1, output_limits=(-5.0, 5.0))
        
        # 过程变量初始化（当前位置）
        self.process_variable = [0.0, 0.0, 0.0]  # [x, y, z]
        self.last_time = self.get_clock().now()  # 初始化时间戳
        self.command_counter = 0  # 命令计数器
        
        # 调试标志
        self.debug_enabled = True  # 设置为False可减少日志输出

    def target_callback(self, msg):
        """目标值回调函数，更新最新消息"""
        self.message_window.append(msg)
        self.latest_message = msg
        
        # 打印接收到的目标值（使用info级别确保可见）
        self.get_logger().info(
            f'Received target #{msg.count}: '
            f'x={msg.target.x:.3f}, y={msg.target.y:.3f}, z={msg.target.z:.3f}'
        )

    def timer_callback(self):
        """定时器回调函数，执行PID控制和消息发布"""
        try:
            if self.latest_message is None:
                self.get_logger().warn('No messages received yet, skipping PID update')
                return
                
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds * 1e-9  # 计算时间差（秒）
            
            # 跳过无效时间差
            if dt <= 0:
                self.get_logger().debug(f'Invalid dt: {dt:.6f}, skipping update')
                self.last_time = current_time
                return
                
            # 提取目标值
            target = [
                self.latest_message.target.x,
                self.latest_message.target.y,
                self.latest_message.target.z
            ]
            
            # 执行PID计算
            output_x = self.pid_x.update(target[0], self.process_variable[0], dt)
            output_y = self.pid_y.update(target[1], self.process_variable[1], dt)
            output_z = self.pid_z.update(target[2], self.process_variable[2], dt)
            
            # 更新过程变量（位置 = 位置 + PID输出 * dt）
            # 注意：乘以dt将速度输出转换为位置增量
            self.process_variable[0] += output_x * dt
            self.process_variable[1] += output_y * dt
            self.process_variable[2] += output_z * dt
            
            # 创建并发布命令消息
            command_msg = Target()
            command_msg.name = "command"
            command_msg.count = self.command_counter
            command_msg.time = current_time.nanoseconds * 1e-9  # 时间戳（秒）
            command_msg.target.x = self.process_variable[0]
            command_msg.target.y = self.process_variable[1]
            command_msg.target.z = self.process_variable[2]
            
            self.publisher_.publish(command_msg)
            self.command_counter += 1
            
            # 打印调试信息
            if self.debug_enabled:
                debug_info = self.pid_x.get_debug_info()
                self.get_logger().info(
                    f'PID Update #{self.command_counter}: '
                    f'Target={debug_info["setpoint"]:.3f}, '
                    f'Current={debug_info["process_value"]:.3f}, '
                    f'Error={debug_info["error"]:.3f}, '
                    f'Output={debug_info["output"]:.3f}, '
                    f'dt={debug_info["dt"]:.6f}'
                )
                
                # 检查是否有警告
                if 'warning' in debug_info:
                    self.get_logger().warn(debug_info['warning'])
            
            # 更新时间戳
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Exception in timer_callback: {str(e)}')
            # 可选：raise e  # 取消注释此行会使节点崩溃，便于调试

def main(args=None):
    rclpy.init(args=args)
    process_node = ProcessNode()
    rclpy.spin(process_node)
    process_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()