#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rh6_msg/msg/rh6_msg.hpp>  // 添加CAN通信的消息类型
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include "rh6_ecat/ryhand_types.hpp"
#include "rh6_ecat/shared_data.h"
#include "rh6_ecat/shm_utils_cpp.hpp"  // 添加共享内存工具类
#include "rh6_ecat/srv/set_hand_mode.hpp"

// 添加ryhandlib.h以获取RyServoInfo结构体定义
#include "ryhandlib.h"

namespace ruiyan::rh6 {

/**
 * @brief ROS 手部接口类 - 处理 ROS 消息和共享内存的转换
 * 
 * 这个类负责：
 * 1. 订阅 ROS 手部控制命令
 * 2. 发布手部状态信息
 * 3. 在 ROS 消息和共享内存数据之间进行转换
 * 4. 提供手部控制的高级接口
 */
class RosHandInterface : public rclcpp::Node {
public:
    RosHandInterface();
    ~RosHandInterface();

    /**
     * @brief 初始化 ROS 接口
     * @return 成功返回 true，失败返回 false
     */
    bool initialize();

    /**
     * @brief 启动 ROS 接口
     */
    void start();

    /**
     * @brief 停止 ROS 接口
     */
    void stop();

    /**
     * @brief 检查接口是否正在运行
     * @return 运行中返回 true，否则返回 false
     */
    bool is_running() const { return running_; }

    /**
     * @brief 设置手部索引
     * @param index 手部索引（0=左手，1=右手）
     */
    void set_hand_index(int index) { hand_index_ = index; }

    /**
     * @brief 设置控制周期
     * @param period_ms 周期时间（毫秒）
     */
    void set_control_period(int period_ms) { period_ms_ = period_ms; }

private:
    // ROS 相关
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_joint_state_pub_;
    
    // EtherCAT状态发布器（与CAN通信格式一致）
    rclcpp::Publisher<rh6_msg::msg::Rh6Msg>::SharedPtr ethercat_state_pub_;
    
    rclcpp::Service<rh6_ecat::srv::SetHandMode>::SharedPtr set_hand_mode_srv_;
    
    // 线程控制
    std::atomic<bool> running_;
    std::thread worker_thread_;
    std::mutex mutex_;
    
    // 手部控制 - 支持双手
    int period_ms_;
    int hand_index_;
    std::vector<double> left_pending_cmd_;
    std::vector<double> right_pending_cmd_;
    
    // 共享内存
    SharedData_t* shared_data_;
    std::unique_ptr<SharedMemoryManager> shm_manager_;
    
    /**
     * @brief 命令回调函数
     * @param msg 手部控制命令消息
     */
    void on_hand_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    /**
     * @brief 左手命令回调函数
     * @param msg 左手控制命令消息
     */
    void on_left_hand_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    /**
     * @brief 右手命令回调函数
     * @param msg 右手控制命令消息
     */
    void on_right_hand_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    /**
     * @brief 设置手部模式服务回调
     * @param request 服务请求
     * @param response 服务响应
     */
    void on_set_hand_mode(
        const std::shared_ptr<rh6_ecat::srv::SetHandMode::Request> request,
        std::shared_ptr<rh6_ecat::srv::SetHandMode::Response> response);
    
    /**
     * @brief 主循环
     */
    void control_loop();
    
    /**
     * @brief 写入手部命令到共享内存
     * @param hand_index 手部索引（0=左手，1=右手）
     */
    void write_hand_command(int hand_index);
    
    /**
     * @brief 从共享内存读取手部状态
     * @param hand_index 手部索引（0=左手，1=右手）
     */
    void read_hand_state(int hand_index);
    
    /**
     * @brief 发布手部状态
     * @param hand_index 手部索引（0=左手，1=右手）
     */
    void publish_hand_state(int hand_index);
    
    /**
     * @brief 发布关节状态
     * @param hand_index 手部索引（0=左手，1=右手）
     */
    void publish_joint_state(int hand_index);
    
    /**
     * @brief 转换位置值
     * @param ros_value ROS 位置值 (0.0-1.0)
     * @return 睿研位置值 (0-4095)
     */
    uint16_t convert_position_to_ryhand(double ros_value);
    
    /**
     * @brief 转换位置值
     * @param ryhand_value 睿研位置值 (0-4095)
     * @return ROS 位置值 (0.0-1.0)
     */
    double convert_position_to_ros(uint16_t ryhand_value);
    
    /**
     * @brief 转换速度值
     * @param ros_value ROS 速度值 (0.0-1.0)
     * @return 睿研速度值 (0-65535)
     */
    uint16_t convert_velocity_to_ryhand(double ros_value);
    
    /**
     * @brief 转换电流值
     * @param ros_value ROS 电流值 (0.0-1.0)
     * @return 睿研电流值 (0-65535)
     */
    uint16_t convert_current_to_ryhand(double ros_value);
    
    /**
     * @brief 发布EtherCAT状态（与CAN通信格式一致）
     */
    void publish_ethercat_state();
    
    /**
     * @brief 从共享内存转换数据到Rh6Msg格式
     * @param msg 输出的Rh6Msg消息
     */
    void convert_shared_data_to_rh6msg(rh6_msg::msg::Rh6Msg& msg);
};

} // namespace ruiyan::rh6
