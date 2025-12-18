#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>

// ROS2 相关
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class HandModeController : public rclcpp::Node {
public:
    HandModeController() : Node("hand_mode_controller") {
        // 创建发布器
        hand_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hand_cmd", 10);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            100ms, std::bind(&HandModeController::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "手部模式控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "请确保 EtherCAT 主站和手部控制节点正在运行");
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::Float64MultiArray();
        
        // 生成正弦波控制信号
        static double time = 0.0;
        time += 0.1; // 100ms 间隔
        
        // 为 6 个关节生成正弦波位置命令
        for (int i = 0; i < 6; i++) {
            double amplitude = 0.5; // 振幅
            double frequency = 0.1; // 频率
            double phase = i * M_PI / 3; // 相位差
            
            double position = amplitude * sin(2 * M_PI * frequency * time + phase);
            message.data.push_back(position);
        }
        
        // 发布消息
        hand_cmd_pub_->publish(message);
        
        // 每 5 秒打印一次状态
        static int counter = 0;
        if (++counter % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "发送控制命令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                message.data[0], message.data[1], message.data[2],
                message.data[3], message.data[4], message.data[5]);
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr hand_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<HandModeController>();
    
    RCLCPP_INFO(node->get_logger(), "开始手部模式控制演示");
    RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 停止");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}