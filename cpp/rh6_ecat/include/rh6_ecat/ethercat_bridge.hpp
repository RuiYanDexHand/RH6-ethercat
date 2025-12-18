#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <string>

#include "rh6_ecat/ryhand_types.hpp"
#include "rh6_ecat/shared_data.h"
#include "rh6_ecat/communication_bridge.hpp"
#include "rh6_ecat/shm_utils_cpp.hpp"

// 添加ryhandlib.h以获取RyServoCmd和RyServoInfo结构体定义
#include "ryhandlib.h"

// 条件编译：只有在找到 EtherCAT 库时才包含 ecrt.h
#ifdef ECAT_LIB_FOUND
extern "C" {
    #include "ecrt.h"
}
#endif

namespace ruiyan::rh6
{
    class ethercat_bridge : public CommunicationBridge
    {
    public:
        ethercat_bridge();
        ~ethercat_bridge() override;

        // 实现基类接口
        bool initialize(const std::string& config = "") override;
        bool start() override;
        void stop() override;
        bool is_running() const override;
        void set_hand_mode(hand_mode_t mode) override;
        void set_hand_index(int index) override;
        std::string get_status() const override;
        void cleanup();
        
        // 实现基类纯虚函数
        CommunicationType get_type() const override;
        int get_slave_count() const override;

        // EtherCAT 相关
#ifdef ECAT_LIB_FOUND
        ec_master_state_t get_master_state() const;
        bool scan_slaves();
        bool configure_slaves();
#endif
        
        // 正弦波控制
        void enable_sine_wave(int mode, double frequency = 0.5, double amplitude = 1500, double offset = 2000);
        void disable_sine_wave();
        void generate_sine_wave_commands();
        void update_motor_parameters();
        void write_sine_wave_to_shared_memory();

    private:
        void cyclic_task();
        void update_shared_memory(SharedData_t* shared_data);
        bool configure_pdos();
        bool register_pdo_entries();
        bool wait_for_op_state();

        // EtherCAT 相关成员
#ifdef ECAT_LIB_FOUND
        ec_master_t* master_;
        ec_domain_t* domain_;
        uint8_t* domain_data_;
        ec_slave_config_t* slave_configs_[32];
        int slave_count_;
        int expected_wkc_;
        ec_master_state_t master_state_;
        int wkc_;  // Working Counter
#endif

        // 线程控制
        std::thread worker_thread_;
        std::atomic<bool> running_;
        std::atomic<bool> initialized_;
        mutable std::mutex mutex_;  // mutable 允许在 const 方法中使用

        // 手部控制参数
        hand_mode_t current_hand_mode_;  // 重命名以匹配实现
        int hand_index_;
        
        // 共享内存
        SharedData_t* shared_data_;  // 添加共享内存指针
        std::unique_ptr<SharedMemoryManager> shm_manager_;  // 共享内存管理器
        
        // 线程管理
        std::thread cyclic_thread_;  // EtherCAT周期线程
        
        // PDO 首项偏移（域注册后赋值，用于循环内读写）
        unsigned int tx_offset_; // 0x7000:1 起始偏移（Master->Slave）
        unsigned int rx_offset_; // 0x6000:1 起始偏移（Slave->Master）
        unsigned int finger_tx_offsets_[6]; // 每个手指的TX偏移量
        unsigned int finger_rx_offsets_[6]; // 每个手指的RX偏移量
        bool minimal_pdo_mode_;  // 当前是否使用最小映射（仅 len 项）
        
        // 正弦波控制参数
        int sine_mode_;                    // 模式：0, 1, 2
        double sine_time_;                 // 时间计数器
        double sine_frequency_;            // 频率 0.5Hz
        double sine_amplitude_;            // 振幅
        double sine_offset_;               // 偏移
        std::atomic<bool> sine_wave_enabled_; // 正弦波使能
        std::chrono::steady_clock::time_point last_command_time_{};
        double command_interval_ms_ = 5.0;
        double last_command_dt_ms_ = 5.0;
        
        // 电机控制参数
        struct MotorParams {
            uint16_t target_position;      // 目标位置 (0-4095)
            uint16_t target_speed;         // 目标速度 (0-4095)
            uint16_t max_current;         // 最大电流 (0-4095)
            uint8_t command;              // 命令 (0xee)
        } motor_params_[6];               // 6个手指的电机参数
        
        // 伺服使能门控（进入OP后先清错/稳定再开始运动）
        bool servo_enabled_ = false;       // 伺服是否已允许运动
        int enable_warmup_cycles_ = 0;     // 预热倒计时（周期数）
        bool op_ready_ = false;            // 是否首次观察到 OP + WC 达标
        
        // 实时优先级设置
        void set_realtime_priority();
        
        // 调试功能函数
        void check_slave_states();
        void verify_pdo_configuration();
        void try_state_transition();
        void print_enhanced_master_state();
        
        // 同步管理器配置获取函数
        ec_sync_info_t* get_ryhand_syncs(int slave_index);
        ec_sync_info_t* get_xnddrive_syncs(int slave_index);
        ec_sync_info_t* get_esc_switch_syncs(int slave_index);
    };

} // namespace ruiyan::rh6