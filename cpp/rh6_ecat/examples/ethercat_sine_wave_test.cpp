#include <rclcpp/rclcpp.hpp>
#include <rh6_msg/msg/rh6_msg.hpp>
#include <rh6_cmd/msg/rh6_cmd.hpp>  // 直接使用rh6_cmd包
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <thread>
#include <cmath>  // 添加cmath以使用sin函数

// 定义M_PI常量（某些编译器可能没有定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class EtherCATSineWaveTest : public rclcpp::Node {
public:
    EtherCATSineWaveTest() : Node("ethercat_sine_wave_test") {
        // 创建发布器 - 直接使用rh6_cmd包
        hand_cmd_pub_ = this->create_publisher<rh6_cmd::msg::Rh6Cmd>("ryhand6_cmd", 10);
        ethercat_state_sub_ = this->create_subscription<rh6_msg::msg::Rh6Msg>(
            "ethercat_hand_status", 10, 
            std::bind(&EtherCATSineWaveTest::on_ethercat_state, this, std::placeholders::_1));
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&EtherCATSineWaveTest::publish_sine_wave_command, this));
        
        RCLCPP_INFO(this->get_logger(), "EtherCAT正弦波测试节点已启动");
    }

private:
    void publish_sine_wave_command() {
        static double time = 0.0;
        static int mode = 0; // 0, 1, 2
        
        // 每5秒切换一次模式
        if (static_cast<int>(time) % 5 == 0 && static_cast<int>(time) != last_mode_time_) {
            mode = (mode + 1) % 3;
            last_mode_time_ = static_cast<int>(time);
            RCLCPP_INFO(this->get_logger(), "切换到模式 %d", mode);
        }
        
        // 直接使用rh6_cmd包中的结构
        rh6_cmd::msg::Rh6Cmd cmd_msg;
        cmd_msg.mode = mode;
        cmd_msg.lr = 0; // 左手
        
        // 生成正弦波命令
        double sine_value = sin(2 * M_PI * 0.5 * time); // 0.5Hz频率
        
        // 根据模式设置不同的字段
        switch (mode) {
            case 0: // 原始位置控制
                for (int i = 0; i < 6; i++) {
                    cmd_msg.m_pos[i] = 0.5 + 0.3 * sine_value; // 0.2-0.8范围
                    cmd_msg.m_spd[i] = 1000.0;  // 速度
                    cmd_msg.m_curlimit[i] = 1000.0; // 电流限制
                }
                break;
                
            case 1: // 角度控制
                for (int i = 0; i < 11; i++) {
                    if (i < 6) {
                        cmd_msg.j_ang[i] = 90.0 + 45.0 * sine_value; // 45-135度
                    } else {
                        cmd_msg.j_ang[i] = 0.0;
                    }
                }
                for (int i = 0; i < 6; i++) {
                    cmd_msg.m_spd[i] = 1000.0;
                    cmd_msg.m_curlimit[i] = 1000.0;
                }
                break;
                
            case 2: // 末端位置控制
                cmd_msg.x_base = 0.0;
                cmd_msg.y_base = 0.0;
                cmd_msg.z_base = 0.0;
                cmd_msg.roll_base = 0.0;
                cmd_msg.pitch_base = 0.0;
                cmd_msg.yaw_base = 0.0;
                
                for (int i = 0; i < 5; i++) {
                    cmd_msg.x[i] = 50.0 + 30.0 * sine_value; // 20-80mm
                    cmd_msg.y[i] = 0.0;
                    cmd_msg.z[i] = 0.0;
                    cmd_msg.roll[i] = 0.0;
                    cmd_msg.pitch[i] = 0.0;
                    cmd_msg.yaw[i] = 0.0;
                }
                for (int i = 0; i < 6; i++) {
                    cmd_msg.m_spd[i] = 1000.0;
                    cmd_msg.m_curlimit[i] = 1000.0;
                }
                break;
        }
        
        hand_cmd_pub_->publish(cmd_msg);
        
        // 打印命令
        if (static_cast<int>(time * 10) % 100 == 0) { // 每1秒打印一次
            RCLCPP_INFO(this->get_logger(), 
                       "模式%d - 时间: %.1fs, 正弦值: %.3f",
                       mode, time, sine_value);
        }
        
        time += 0.01; // 10ms周期
    }
    
    void on_ethercat_state(const rh6_msg::msg::Rh6Msg::SharedPtr msg) {
        // 打印接收到的状态
        RCLCPP_INFO(this->get_logger(), 
                   "EtherCAT状态 - 位置: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
                   "速度: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], "
                   "电流: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                   msg->m_pos[0], msg->m_pos[1], msg->m_pos[2], msg->m_pos[3], msg->m_pos[4], msg->m_pos[5],
                   msg->m_spd[0], msg->m_spd[1], msg->m_spd[2], msg->m_spd[3], msg->m_spd[4], msg->m_spd[5],
                   msg->m_cur[0], msg->m_cur[1], msg->m_cur[2], msg->m_cur[3], msg->m_cur[4], msg->m_cur[5]);
    }
    
    rclcpp::Publisher<rh6_cmd::msg::Rh6Cmd>::SharedPtr hand_cmd_pub_;  // 使用rh6_cmd包
    rclcpp::Subscription<rh6_msg::msg::Rh6Msg>::SharedPtr ethercat_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int last_mode_time_ = -1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<EtherCATSineWaveTest>();
    
    RCLCPP_INFO(node->get_logger(), "EtherCAT正弦波测试开始...");
    RCLCPP_INFO(node->get_logger(), "模式0: 原始位置控制 (0.2-0.8)");
    RCLCPP_INFO(node->get_logger(), "模式1: 角度控制 (36-144度)");
    RCLCPP_INFO(node->get_logger(), "模式2: 末端位置控制 (20-80mm)");
    RCLCPP_INFO(node->get_logger(), "每5秒自动切换模式");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
