#include <rclcpp/rclcpp.hpp>
#include <driver_msgs/msg/target.hpp>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node
{
public:
    DriverNode() : Node("driver_node")
    {
        // 声明并获取参数
        declare_parameter("initial_count", 0);
        initial_count_ = get_parameter("initial_count").as_int();
        
        // 检查参数范围
        if (initial_count_ < 0 || initial_count_ > 100) {
            RCLCPP_WARN(get_logger(), "Initial count %d is out of range [0, 100]", initial_count_);
        }
        
        count_ = initial_count_;
        
        // 更新发布者消息类型
        publisher_ = this->create_publisher<driver_msgs::msg::Target>("target", 10);
        
        // 创建定时器 (500Hz = 2ms周期)
        timer_ = this->create_wall_timer(
            2ms, std::bind(&DriverNode::timer_callback, this));
        
        // 记录节点启动时间
        start_time_ = this->now();
    }

private:
    void timer_callback()
    {
        count_++;
        
        // 创建并填充自定义消息
        auto message = driver_msgs::msg::Target();
        message.name = "targ";
        message.count = count_;
        
        // 计算时间
        auto current_time = this->now();
        auto elapsed_time = (current_time - start_time_).seconds();
        message.time = elapsed_time;
        
        // 计算向量值
        message.target.x = std::sin(elapsed_time);
        message.target.y = std::cos(elapsed_time);
        message.target.z = std::sin(2 * elapsed_time);
        
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Message %d published at %.2f seconds", count_, elapsed_time);
    }
    
    int count_;
    int initial_count_;
    rclcpp::Publisher<driver_msgs::msg::Target>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}