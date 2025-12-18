#include "rh6_ecat/ros_hand_interface.hpp"
#include "rh6_ecat/shm_utils_cpp.hpp"
#include "rh6_ecat/ethercat_slave_config.h"
#include "ryhandlib.h"  // 添加ryhandlib.h以获取FingerServoInfo_t定义
#include <chrono>
#include <algorithm>
#include <cmath>

// 定义M_PI常量（某些编译器可能没有定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using std::placeholders::_1;

namespace ruiyan::rh6 {

RosHandInterface::RosHandInterface() 
    : Node("ros_hand_interface")
    , running_(false)
    , period_ms_(10)
    , shared_data_(nullptr) {
    
    // 声明参数
    period_ms_ = this->declare_parameter<int>("period_ms", 10);
    
    // 初始化共享内存
    try {
        shm_manager_ = SharedMemoryUtils::create_manager(false, "/ethercat_data");
        if (!shm_manager_ || !shm_manager_->is_valid()) {
            RCLCPP_FATAL(this->get_logger(), "Shared memory initialization failed");
            throw std::runtime_error("shared memory init failed");
        }
        shared_data_ = shm_manager_->get();
        if (!shared_data_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get shared data pointer");
            throw std::runtime_error("shared data pointer failed");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to shared memory");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Shared memory error: %s", e.what());
        throw;
    }
}

RosHandInterface::~RosHandInterface() {
    stop();
}

bool RosHandInterface::initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing ROS Hand Interface...");
    
    // 创建订阅者 - 支持双手
    left_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "left_hand_cmd", 10, std::bind(&RosHandInterface::on_left_hand_command, this, _1));
    right_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "right_hand_cmd", 10, std::bind(&RosHandInterface::on_right_hand_command, this, _1));
    
    // 创建发布者 - 支持双手
    left_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("left_hand_state", 10);
    right_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("right_hand_state", 10);
    left_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("left_joint_states", 10);
    right_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("right_joint_states", 10);
    
    // 创建EtherCAT状态发布器（与CAN通信格式一致）
    ethercat_state_pub_ = this->create_publisher<rh6_msg::msg::Rh6Msg>("ethercat_hand_status", 10);
    
    // 创建服务
    set_hand_mode_srv_ = this->create_service<rh6_ecat::srv::SetHandMode>(
        "set_hand_mode", 
        std::bind(&RosHandInterface::on_set_hand_mode, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "ROS Hand Interface initialized successfully");
    return true;
}

void RosHandInterface::start() {
    if (running_) {
        RCLCPP_WARN(this->get_logger(), "ROS Hand Interface is already running");
        return;
    }
    
    running_ = true;
    worker_thread_ = std::thread(&RosHandInterface::control_loop, this);
    
    RCLCPP_INFO(this->get_logger(), "ROS Hand Interface started");
}

void RosHandInterface::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    
    RCLCPP_INFO(this->get_logger(), "ROS Hand Interface stopped");
}

void RosHandInterface::on_left_hand_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 验证命令数据
    if (msg->data.size() > 6) {
        RCLCPP_WARN(this->get_logger(), "Left hand command data size too large: %zu, expected <= 6", msg->data.size());
    }
    
    // 限制数据大小并验证范围
    left_pending_cmd_.clear();
    for (size_t i = 0; i < std::min(msg->data.size(), size_t(6)); i++) {
        double value = std::clamp(msg->data[i], 0.0, 1.0);
        left_pending_cmd_.push_back(value);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received left hand command with %zu values", left_pending_cmd_.size());
}

void RosHandInterface::on_right_hand_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 验证命令数据
    if (msg->data.size() > 6) {
        RCLCPP_WARN(this->get_logger(), "Right hand command data size too large: %zu, expected <= 6", msg->data.size());
    }
    
    // 限制数据大小并验证范围
    right_pending_cmd_.clear();
    for (size_t i = 0; i < std::min(msg->data.size(), size_t(6)); i++) {
        double value = std::clamp(msg->data[i], 0.0, 1.0);
        right_pending_cmd_.push_back(value);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Received right hand command with %zu values", right_pending_cmd_.size());
}

void RosHandInterface::control_loop() {
    RCLCPP_INFO(this->get_logger(), "ROS Hand Interface control loop started");
    
    rclcpp::WallRate rate{std::chrono::milliseconds(period_ms_)};
    
    while (running_ && rclcpp::ok()) {
        // 写入手部命令 - 支持双手
        write_hand_command(0);  // 左手
        write_hand_command(1);  // 右手
        
        // 读取手部状态 - 支持双手
        read_hand_state(0);     // 左手
        read_hand_state(1);     // 右手
        
        // 发布状态信息 - 支持双手
        publish_hand_state(0);  // 左手
        publish_hand_state(1);  // 右手
        publish_joint_state(0); // 左手
        publish_joint_state(1); // 右手
        
        // 发布EtherCAT状态（与CAN通信格式一致）
        publish_ethercat_state();
        
        rate.sleep();
    }
    
    RCLCPP_INFO(this->get_logger(), "ROS Hand Interface control loop stopped");
}

void RosHandInterface::write_hand_command(int hand_index) {
    if (!shared_data_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 根据手部索引选择对应的命令数据
    std::vector<double>* cmd_data = nullptr;
    if (hand_index == 0) {
        if (left_pending_cmd_.empty()) return;
        cmd_data = &left_pending_cmd_;
    } else if (hand_index == 1) {
        if (right_pending_cmd_.empty()) return;
        cmd_data = &right_pending_cmd_;
    } else {
        return; // 无效的手部索引
    }
    
    pthread_mutex_lock(&shared_data_->mutex);
    
    // 将手部命令转换为睿研格式并写入共享内存
    for (size_t i = 0; i < std::min(cmd_data->size(), size_t(6)); i++) {
        RyServoCmd servo_cmd;
        servo_cmd.cmd = 0xee;
        servo_cmd.usPos = convert_position_to_ryhand((*cmd_data)[i]);
        servo_cmd.usSpd = 2000;  // 默认速度
        servo_cmd.usMaxCur = 1000;  // 默认电流
        servo_cmd.res = 0;
        
        // 写入共享内存的 RyHand 数据区域
        shared_data_->ryhand[hand_index].tx_len[i] = sizeof(RyServoCmd);
        memcpy(shared_data_->ryhand[hand_index].tx_data[i], &servo_cmd, sizeof(RyServoCmd));
    }
    
    // 更新发送数据计数
    shared_data_->tx_data_cnt++;
    shared_data_->tx_nh = std::min(static_cast<int>(cmd_data->size()), 6);
    
    pthread_mutex_unlock(&shared_data_->mutex);
}

void RosHandInterface::read_hand_state(int hand_index) {
    if (!shared_data_) {
        return;
    }
    
    pthread_mutex_lock(&shared_data_->mutex);
    
    // 从共享内存的 RyHand 数据区域读取状态
    for (int i = 0; i < 6; i++) {
        if (shared_data_->ryhand[hand_index].rx_len[i] >= sizeof(FingerServoInfo_t)) {
            FingerServoInfo_t* info = (FingerServoInfo_t*)shared_data_->ryhand[hand_index].rx_data[i];
            // 这里可以处理接收到的状态信息
            // 例如：info->ub_P (位置), info->ub_V (速度), info->ub_I (电流)
        }
    }
    
    pthread_mutex_unlock(&shared_data_->mutex);
}

void RosHandInterface::publish_hand_state(int hand_index) {
    if (!shared_data_) {
        return;
    }
    
    std_msgs::msg::Float64MultiArray state_msg;
    state_msg.data.resize(6);
    
    pthread_mutex_lock(&shared_data_->mutex);
    
    // 从共享内存的 RyHand 数据区域读取状态
    for (int i = 0; i < 6; i++) {
        if (shared_data_->ryhand[hand_index].rx_len[i] >= sizeof(FingerServoInfo_t)) {
            FingerServoInfo_t* info = (FingerServoInfo_t*)shared_data_->ryhand[hand_index].rx_data[i];
            // 转换位置 (0-4095 -> 0.0-1.0)
            state_msg.data[i] = convert_position_to_ros(info->ub_P);
        } else {
            state_msg.data[i] = 0.0;
        }
    }
    
    pthread_mutex_unlock(&shared_data_->mutex);
    
    // 根据手部索引发布到对应话题
    if (hand_index == 0) {
        left_state_pub_->publish(state_msg);
    } else if (hand_index == 1) {
        right_state_pub_->publish(state_msg);
    }
}

void RosHandInterface::publish_joint_state(int hand_index) {
    if (!shared_data_) {
        return;
    }
    
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.header.frame_id = hand_index == 0 ? "left_hand_base" : "right_hand_base";
    
    // 设置关节名称
    std::string prefix = hand_index == 0 ? "left_" : "right_";
    joint_state_msg.name = {
        prefix + "finger_1_joint", prefix + "finger_2_joint", prefix + "finger_3_joint",
        prefix + "finger_4_joint", prefix + "finger_5_joint", prefix + "finger_6_joint"
    };
    
    joint_state_msg.position.resize(6);
    joint_state_msg.velocity.resize(6);
    joint_state_msg.effort.resize(6);
    
    pthread_mutex_lock(&shared_data_->mutex);
    
    // 从共享内存读取关节状态
    for (int i = 0; i < 6; i++) {
        if (shared_data_->ryhand[hand_index].rx_len[i] >= sizeof(FingerServoInfo_t)) {
            FingerServoInfo_t* info = (FingerServoInfo_t*)shared_data_->ryhand[hand_index].rx_data[i];
            
            // 转换位置 (0-4095 -> 弧度)
            joint_state_msg.position[i] = convert_position_to_ros(info->ub_P) * M_PI / 2.0;
            
            // 转换速度 (0-65535 -> 弧度/秒)
            joint_state_msg.velocity[i] = static_cast<double>(info->ub_V) / 65535.0 * 10.0;
            
            // 转换扭矩 (使用电流 I 作为扭矩指示)
            joint_state_msg.effort[i] = static_cast<double>(info->ub_I) / 2048.0 * 5.0;
        } else {
            joint_state_msg.position[i] = 0.0;
            joint_state_msg.velocity[i] = 0.0;
            joint_state_msg.effort[i] = 0.0;
        }
    }
    
    pthread_mutex_unlock(&shared_data_->mutex);
    
    // 根据手部索引发布到对应话题
    if (hand_index == 0) {
        left_joint_state_pub_->publish(joint_state_msg);
    } else if (hand_index == 1) {
        right_joint_state_pub_->publish(joint_state_msg);
    }
}

uint16_t RosHandInterface::convert_position_to_ryhand(double ros_value) {
    return static_cast<uint16_t>(std::clamp(ros_value, 0.0, 1.0) * 4095.0);
}

double RosHandInterface::convert_position_to_ros(uint16_t ryhand_value) {
    return static_cast<double>(ryhand_value) / 4095.0;
}

uint16_t RosHandInterface::convert_velocity_to_ryhand(double ros_value) {
    return static_cast<uint16_t>(std::clamp(ros_value, 0.0, 1.0) * 65535.0);
}

uint16_t RosHandInterface::convert_current_to_ryhand(double ros_value) {
    return static_cast<uint16_t>(std::clamp(ros_value, 0.0, 1.0) * 65535.0);
}

void RosHandInterface::on_set_hand_mode(
    const std::shared_ptr<rh6_ecat::srv::SetHandMode::Request> request,
    std::shared_ptr<rh6_ecat::srv::SetHandMode::Response> response) {
    
    // 验证参数
    if (request->hand_index < 0 || request->hand_index >= 2) {
        response->success = false;
        response->message = "Invalid hand index. Must be 0 (left) or 1 (right)";
        return;
    }
    
    if (request->mode < 0 || request->mode > 2) {
        response->success = false;
        response->message = "Invalid mode. Must be 0 (position), 1 (velocity), or 2 (torque)";
        return;
    }
    
    // 设置手部模式
    hand_mode_t mode = static_cast<hand_mode_t>(request->mode);
    if (set_hand_mode(request->hand_index, mode) == 0) {
        response->success = true;
        response->message = "Hand mode set successfully";
        
        // 记录模式切换
        const char* mode_names[] = {"position", "velocity", "torque"};
        const char* hand_names[] = {"left", "right"};
        RCLCPP_INFO(this->get_logger(), 
                   "Set %s hand to %s control mode", 
                   hand_names[request->hand_index], 
                   mode_names[request->mode]);
    } else {
        response->success = false;
        response->message = "Failed to set hand mode";
    }
}

void RosHandInterface::publish_ethercat_state() {
    rh6_msg::msg::Rh6Msg state_msg;
    convert_shared_data_to_rh6msg(state_msg);
    ethercat_state_pub_->publish(state_msg);
}

void RosHandInterface::convert_shared_data_to_rh6msg(rh6_msg::msg::Rh6Msg& msg) {
    if (!shared_data_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 直接使用rh6_msg包中的结构，无需重复定义
    msg.lr = 0; // 0=左手, 1=右手
    
    // 从共享内存读取6个手指的数据
    for (int i = 0; i < 6; i++) {
        if (shared_data_->ryhand[0].rx_len[i] >= sizeof(FingerServoInfo_t)) {
            FingerServoInfo_t* info = (FingerServoInfo_t*)shared_data_->ryhand[0].rx_data[i];
            
            // 直接使用rh6_msg包中的字段
            msg.status[i] = info->ucStatus;         // uint8[6] status
            msg.m_pos[i] = info->ub_P;              // float32[6] m_pos
            msg.m_spd[i] = info->ub_V;              // float32[6] m_spd
            msg.m_cur[i] = info->ub_I;              // float32[6] m_cur
            msg.m_force[i] = info->ub_F;            // float32[6] m_force
        } else {
            msg.status[i] = 0;
            msg.m_pos[i] = 0.0;
            msg.m_spd[i] = 0.0;
            msg.m_cur[i] = 0.0;
            msg.m_force[i] = 0.0;
        }
    }
    
    // 使用rh6_msg包中的关节角度字段
    for (int i = 0; i < 11; i++) {
        if (i < 6) {
            msg.j_ang[i] = (msg.m_pos[i] / 4095.0) * 180.0; // float32[11] j_ang
        } else {
            msg.j_ang[i] = 0.0;
        }
    }
    
    // 使用rh6_msg包中的基坐标系字段
    msg.x_base = 0.0;      // float64 x_base
    msg.y_base = 0.0;      // float64 y_base
    msg.z_base = 0.0;      // float64 z_base
    msg.roll_base = 0.0;   // float64 roll_base
    msg.pitch_base = 0.0;  // float64 pitch_base
    msg.yaw_base = 0.0;    // float64 yaw_base
    
    // 使用rh6_msg包中的末端位置字段
    for (int i = 0; i < 5; i++) {
        msg.x[i] = 0.0;    // float64[5] x
        msg.y[i] = 0.0;    // float64[5] y
        msg.z[i] = 0.0;    // float64[5] z
        msg.roll[i] = 0.0; // float64[5] roll
        msg.pitch[i] = 0.0;// float64[5] pitch
        msg.yaw[i] = 0.0;  // float64[5] yaw
        msg.w[i] = 1.0;    // float64[5] w
        msg.i[i] = 0.0;    // float64[5] i
        msg.j[i] = 0.0;    // float64[5] j
        msg.k[i] = 0.0;    // float64[5] k
    }
}

} // namespace ruiyan::rh6
