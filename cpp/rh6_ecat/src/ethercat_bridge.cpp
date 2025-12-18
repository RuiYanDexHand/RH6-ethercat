#include "rh6_ecat/ethercat_bridge.hpp"
#include "rh6_ecat/shared_data.h"
#include "rh6_ecat/shm_utils_cpp.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <cstring>
#include <cmath>  // 添加cmath以使用sin函数
#include <algorithm>  // 添加algorithm以使用std::max和std::min
#include <limits>

// 定义M_PI常量（某些编译器可能没有定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FREQUENCY 1000
#define PERIOD_NS (1000000000 / FREQUENCY)  // 1ms = 1000000ns

namespace ruiyan::rh6
{

#ifdef ECAT_LIB_FOUND
extern "C" {
    #include <ecrt.h>
}
#include <sys/mman.h>
#include <sched.h>
#endif

    ethercat_bridge::ethercat_bridge() : CommunicationBridge()
    {
        RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "Creating ethercat communication bridge...");
        

        master_ = nullptr;
        domain_ = nullptr;
        domain_data_ = nullptr;
        slave_count_ = 0;
        expected_wkc_ = 0;
        tx_offset_ = 0;
        rx_offset_ = 0;
        initialized_ = false;
        running_ = false;
        
        // 初始化正弦波控制参数（使用更大的振幅）
        sine_mode_ = 0;
        sine_time_ = 0.0;
        sine_frequency_ = 0.5;
        sine_amplitude_ = 1500;  // 增加到2500，使动作更明显
        sine_offset_ = 1500;     // 偏移也设为2500
        sine_wave_enabled_ = false;
        last_command_time_ = std::chrono::steady_clock::now();
        last_command_dt_ms_ = command_interval_ms_;
        
        // 初始化电机参数（电机0和1初始化为0，其他为2000）
        for (int i = 0; i < 6; i++) {
            if (i <= 1) {
                motor_params_[i].target_position = 0;  // 电机0和1保持静止
                motor_params_[i].target_speed = 0;
            } else {
                motor_params_[i].target_position = 2000;
                motor_params_[i].target_speed = 1000;
            }
            motor_params_[i].max_current = 1000;
            motor_params_[i].command = 0xee;
        }
    
        // 初始化共享内存管理器 - 创建新的共享内存对象
        try {
            shm_manager_ = std::make_unique<SharedMemoryManager>(true); // true表示创建新的
            RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器创建成功");
        } catch (const std::exception& e) {
            //RCLCPP_ERROR(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器创建失败: %s", e.what());
            //RCLCPP_WARN(rclcpp::get_logger("ethercat_bridge"), "尝试清理可能存在的旧共享内存...");
            
            // 尝试清理旧的共享内存
            system("rm -f /dev/shm/ethercat_data");
            
            // 再次尝试创建
            try {
                shm_manager_ = std::make_unique<SharedMemoryManager>(true);
              //  RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器创建成功（清理后）");
            } catch (const std::exception& e2) {
                //RCLCPP_ERROR(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器创建仍然失败: %s", e2.what());
                // 构造函数不能返回值，设置shm_manager_为nullptr
                shm_manager_ = nullptr;
            }
        }
    
    // 初始化从站配置数组
    for (int i = 0; i < 32; i++) {
        slave_configs_[i] = nullptr;
    }
#endif
}

    ethercat_bridge::~ethercat_bridge()
    {
    stop();
    cleanup();
}

    bool ethercat_bridge::initialize(const std::string& config)
    {
        RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "=== EtherCAT Bridge 初始化开始 ===");
        //RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "按照ryhand6_ncb.c结构进行初始化...");
    

        // 1. 初始化共享内存 (对应ryhand6_ncb.c的create_shared_memory)
        //RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "1. 初始化共享内存...");
        if (!shm_manager_) {
          //  RCLCPP_ERROR(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器未初始化");
            return false;
        }
        if (!shm_manager_->is_valid()) {
            //RCLCPP_ERROR(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器无效");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "共享内存管理器创建成功");
        
        shared_data_ = shm_manager_->get();
        if (!shared_data_) {
           // RCLCPP_ERROR(rclcpp::get_logger("ethercat_bridge"), "无法获取共享数据指针");
            return false;
        }
        //RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "共享数据指针获取成功: %p", static_cast<void*>(shared_data_));
        
        // 初始化共享数据结构（对应ryhand6_ncb.c的memset）
        memset(shared_data_, 0, sizeof(SharedData_t));
        RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "共享内存初始化完成");
    
    // 2. 请求 EtherCAT 主站
    std::cout << "2. 请求 EtherCAT 主站..." << std::endl;
    master_ = ecrt_request_master(0);
    if (!master_) {
        std::cerr << "   [ERROR] EtherCAT 主站请求失败" << std::endl;
        return false;
    }
    std::cout << "   [OK] EtherCAT 主站请求成功: " << static_cast<void*>(master_) << std::endl;
    
    // 3. 创建 EtherCAT 域
    std::cout << "3. 创建 EtherCAT 域..." << std::endl;
    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        std::cerr << "   [ERROR] EtherCAT 域创建失败" << std::endl;
        return false;
    }
    std::cout << "   [OK] EtherCAT 域创建成功: " << static_cast<void*>(domain_) << std::endl;
    
    // 4. 扫描 EtherCAT 从站 (对应ryhand6_ncb.c的从站扫描)
    std::cout << "4. 扫描 EtherCAT 从站..." << std::endl;
    
    // 从主站中得到从站个数 (对应ryhand6_ncb.c的ecrt_master)
    ec_master_info_t master_info;
    int slave_count = 0;
    if (ecrt_master(master_, &master_info)) {
        std::cerr << "   [ERROR] Failed to get master info." << std::endl;
        return false;
    }
    
    slave_count = master_info.slave_count;
    //std::cout << "   检测到从站个数: " << slave_count << std::endl;
    
    // 计算期望的工作计数器(WC)值 (对应ryhand6_ncb.c的WC计算)
    expected_wkc_ = 0;
    for (int i = 0; i < slave_count; i++) {
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            continue;
        }
        
        // 根据从站类型计算期望的WC值
        if (strcmp(slave_info.name, "ryhand") == 0) {
            expected_wkc_ += 3; // ryhand有输入和输出PDO，各贡献1个WC
            if (shared_data_) {
                shared_data_->rx_nh++;
            }
        }
        else if (strcmp(slave_info.name, "XNDDrive") == 0) {
            expected_wkc_ += 3; // XNDDrive有输入和输出PDO，各贡献1个WC
            if (shared_data_) {
                shared_data_->rx_nj++;
            }
        }
        // ESC Switch通常不参与PDO通信，不增加WC
    }
    
    std::cout << "   期望的工作计数器(WC)值: " << expected_wkc_ << std::endl;
    if (shared_data_) {
        shared_data_->master_status.expected_wc = expected_wkc_;
    }
    
    std::cout << "   [OK] 从站扫描完成" << std::endl;
    
    // 5. 配置 EtherCAT 从站 (对应ryhand6_ncb.c的从站配置)
    std::cout << "5. 配置 EtherCAT 从站..." << std::endl;
    //std::cout << "   需要配置 " << slave_count << " 个从站" << std::endl;
    
    for (int i = 0; i < slave_count; i++) {
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "   [ERROR] Failed to get slave " << i << " info." << std::endl;
            continue;
        }
        //std::cout << "   从站 " << i << " 名称: " << slave_info.name << std::endl;
        
        // 根据从站名字进行配置 (对应ryhand6_ncb.c的从站配置)
        if (strcmp(slave_info.name, "ryhand") == 0) {
            //std::cout << "   配置ryhand从站..." << std::endl;
            
            slave_configs_[i] = ecrt_master_slave_config(master_, 0, i, slave_info.vendor_id, slave_info.product_code);
            if (!slave_configs_[i]) {
                std::cerr << "   [ERROR] Failed to get slave configuration." << std::endl;
                return false;
            }
            
            // 配置PDO (对应ryhand6_ncb.c的ecrt_slave_config_pdos)
            //std::cout << "   配置PDO..." << std::endl;
            if (ecrt_slave_config_pdos(slave_configs_[i], EC_END, NULL)) {
                std::cerr << "   [ERROR] Failed to configure PDOs." << std::endl;
                return false;
            }
            //std::cout << "   [OK] PDO配置成功" << std::endl;
            
            // 配置DC (对应ryhand6_ncb.c的ecrt_slave_config_dc)
            //std::cout << "   配置DC..." << std::endl;
            ecrt_slave_config_dc(slave_configs_[i], 0x0300, PERIOD_NS, PERIOD_NS*2/10, 0, 0);
            //std::cout << "   [OK] DC配置成功" << std::endl;
            
            //std::cout << "   [OK] 从站 " << i << " 配置成功" << std::endl;
        }
        else if (strcmp(slave_info.name, "XNDDrive") == 0) {
            //std::cout << "   配置XNDDrive从站..." << std::endl;
            
            slave_configs_[i] = ecrt_master_slave_config(master_, 0, i, slave_info.vendor_id, slave_info.product_code);
            if (!slave_configs_[i]) {
                std::cerr << "   [ERROR] Failed to get slave configuration." << std::endl;
                return false;
            }
            
            // 暂时禁用PDO配置 - 用于调试
            //std::cout << "   [DEBUG] 暂时跳过PDO配置..." << std::endl;
            
            std::cout << "   [OK] 从站 " << i << " 配置成功" << std::endl;
        }
        else {
            //std::cout << "   未知从站类型: " << slave_info.name << std::endl;
            slave_configs_[i] = ecrt_master_slave_config(master_, 0, i, slave_info.vendor_id, slave_info.product_code);
            if (!slave_configs_[i]) {
                std::cerr << "   [ERROR] Failed to get slave configuration." << std::endl;
                return false;
            }
            std::cout << "   [OK] 从站 " << i << " 配置成功（默认配置）" << std::endl;
        }
    }
    
    //std::cout << "   [OK] " << slave_count << "/" << slave_count << " 个从站配置完成" << std::endl;
    std::cout << "   [OK] 从站配置完成" << std::endl;
    
    // 5.1 配置PDO条目注册 (对应ryhand6_ncb.c的ecrt_domain_reg_pdo_entry_list)
    //std::cout << "5.1 配置PDO条目注册..." << std::endl;
    
    // 使用单从站ryhand PDO条目注册 (对应ryhand6_ncb.c的rhand_single_regs)
    if (slave_count == 1) {
      //  std::cout << "   使用单从站ryhand PDO条目注册..." << std::endl;
        
        // 查找ryhand从站的索引
        int ryhand_slave_pos = -1;
        for (int i = 0; i < slave_count; i++) {
            ec_slave_info_t slave_info;
            if (ecrt_master_get_slave(master_, i, &slave_info)) {
                continue;
            }
            if (strcmp(slave_info.name, "ryhand") == 0) {
                ryhand_slave_pos = i;
                break;
            }
        }
        
        if (ryhand_slave_pos < 0 || ryhand_slave_pos >= 32 || !slave_configs_[ryhand_slave_pos]) {
            std::cerr << "   [ERROR] 未找到有效的ryhand从站配置" << std::endl;
            return false;
        }
        
        //std::cout << "   找到ryhand从站，位置: " << ryhand_slave_pos << std::endl;
        
        // 定义偏移量数组 (对应ryhand6_ncb.c的off_out和off_in)
        static unsigned int off_out[40] = {0};
        static unsigned int off_in[40] = {0};
        
        // 定义ryhand单从站的PDO条目（配置6个手指，对应cstruct.c的slave_0_pdo_entries）
        // 使用固定的vendor_id和product_code（0x00002A3F, 0x00050004）
        // 每个TxData/RxData是192位（24字节）
        ec_pdo_entry_reg_t rhand_multi_regs[] = {
            // 输出PDO (Master->Slave) - 6个手指的Txlen和TxData
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 1, &off_out[0]},   // Txlen_1
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 2, &off_out[1]},  // TxData_1
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 3, &off_out[2]},   // Txlen_2
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 4, &off_out[3]},  // TxData_2
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 5, &off_out[4]},   // Txlen_3
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 6, &off_out[5]},  // TxData_3
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 7, &off_out[6]},   // Txlen_4
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 8, &off_out[7]},  // TxData_4
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 9, &off_out[8]},   // Txlen_5
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 10, &off_out[9]},  // TxData_5
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 11, &off_out[10]},  // Txlen_6
            {0, 0, 0x00002A3F, 0x00050004, 0x7000, 12, &off_out[11]}, // TxData_6
            
            // 输入PDO (Slave->Master) - 6个手指的Rxlen和RxData
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 1, &off_in[0]},     // Rxlen_1
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 2, &off_in[1]},   // RxData_1
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 3, &off_in[2]},   // Rxlen_2
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 4, &off_in[3]},   // RxData_2
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 5, &off_in[4]},   // Rxlen_3
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 6, &off_in[5]},   // RxData_3
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 7, &off_in[6]},   // Rxlen_4
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 8, &off_in[7]},   // RxData_4
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 9, &off_in[8]},   // Rxlen_5
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 10, &off_in[9]},  // RxData_5
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 11, &off_in[10]},  // Rxlen_6
            {0, 0, 0x00002A3F, 0x00050004, 0x6000, 12, &off_in[11]}, // RxData_6
            {}
        };
        
        // PDO条目注册
        if (ecrt_domain_reg_pdo_entry_list(domain_, rhand_multi_regs)) {
            std::cerr << "   [ERROR] PDO entry registration failed!" << std::endl;
            return false;
        }
        //std::cout << "   [OK] PDO条目注册成功（6个手指）" << std::endl;
        
        // 保存每个手指的偏移量
        // 每个手指有2个偏移：Txlen和TxData
        for (int i = 0; i < 6; i++) {
            finger_tx_offsets_[i] = off_out[i * 2];     // Txlen
            finger_rx_offsets_[i] = off_in[i * 2];     // Rxlen
        }
        tx_offset_ = off_out[0];  // 第一个手指的Txlen偏移
        rx_offset_ = off_in[0];   // 第一个手指的Rxlen偏移
        
        //std::cout << "   TX/RX偏移量:" << std::endl;
        for (int i = 0; i < 6; i++) {
            std::cout << "     手指" << i << ": TX偏移=" << finger_tx_offsets_[i] 
                      << ", RX偏移=" << finger_rx_offsets_[i] << std::endl;
        }
    } else {
        //std::cerr << "   [ERROR] 多从站配置暂不支持" << std::endl;
        return false;
    }
    
    std::cout << "   [OK] PDO配置完成" << std::endl;
    
    // 6. 激活 EtherCAT 主站 (对应ryhand6_ncb.c的ecrt_master_activate)
    //std::cout << "6. 激活 EtherCAT 主站..." << std::endl;
    if (ecrt_master_activate(master_) != 0) {
        std::cerr << "   [ERROR] Failed to activate master." << std::endl;
        return false;
    }
    std::cout << "   [OK] EtherCAT 主站激活成功" << std::endl;
    
    // 7. 获取域数据指针 (对应ryhand6_ncb.c的ecrt_domain_data)
    //std::cout << "7. 获取域数据指针..." << std::endl;
        domain_data_ = ecrt_domain_data(domain_);
        if (!domain_data_) {
        std::cerr << "   [ERROR] Failed to get domain1 process data pointer." << std::endl;
            return false;
    }
    
    // 获取域数据大小
    size_t domain_size = ecrt_domain_size(domain_);
    //std::cout << "   [OK] 域数据指针获取成功: " << static_cast<void*>(domain_data_) << std::endl;
    //std::cout << "   [INFO] 域数据大小: " << domain_size << " 字节" << std::endl;
    
    // 显示PDO条目信息
    //std::cout << "   [INFO] PDO条目偏移量信息:" << std::endl;
    //std::cout << "     TX偏移量: " << tx_offset_ << ", RX偏移量: " << rx_offset_ << std::endl;
    
    // 8. 设置实时优先级 (对应ryhand6_ncb.c的sched_setscheduler)
    //std::cout << "8. 设置实时优先级..." << std::endl;
    set_realtime_priority();
    //std::cout << "   [OK] 实时优先级设置完成" << std::endl;
    
    // 8.1 设置主线程实时优先级（与ryhand6_ncb.c完全一致）
    //std::cout << "8.1 设置主线程实时优先级..." << std::endl;
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // 最高优先级，与ryhand6_ncb.c一致
    
    //std::cout << "Using priority " << param.sched_priority << ".";
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        std::cout << " [WARN] 主线程实时优先级设置失败，继续运行..." << std::endl;
        perror("sched_setscheduler failed");
    } else {
        std::cout << " [OK] 主线程实时优先级设置成功" << std::endl;
    }
    
    // 8.2 设置CPU亲和性（避免ROS2线程干扰）
    //std::cout << "8.2 设置CPU亲和性..." << std::endl;
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset); // 绑定到CPU 0
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) == -1) {
        std::cout << "   [WARN] CPU亲和性设置失败，继续运行..." << std::endl;
        perror("sched_setaffinity failed");
    } else {
        std::cout << "   [OK] CPU亲和性设置成功（绑定到CPU 0）" << std::endl;
    }
    
    // 9. 启动周期线程
    //std::cout << "9. 启动周期线程..." << std::endl;
    running_ = true; // 在启动周期线程前设置运行状态
    cyclic_thread_ = std::thread(&ethercat_bridge::cyclic_task, this);
    std::cout << "   [OK] 周期线程启动完成" << std::endl;
    
    // 10. 等待从站进入 OP 状态 (对应ryhand6_ncb.c的状态检查)
    //std::cout << "10. 等待从站进入 OP 状态..." << std::endl;
    //std::cout << "   最大等待时间: 120 秒" << std::endl;
    //std::cout << "   注意：周期任务已启动，数据交换将持续进行..." << std::endl;
    
    // 等待从站进入OP状态
    int wait_cycles = 0;
    int max_wait_cycles = 120000; // 120秒，每1ms一个周期 (EtherCAT从站需要更长时间)
    
    while (wait_cycles < max_wait_cycles) {
        // 检查主站状态
        ec_master_state_t master_state;
        ecrt_master_state(master_, &master_state);
        
        // 检查域状态
        ec_domain_state_t domain_state;
        ecrt_domain_state(domain_, &domain_state);
        
        if (wait_cycles % 1000 == 0) { // 每1秒打印一次
            //std::cout << "   等待中... 周期 " << wait_cycles 
            //          << ", WC=" << domain_state.working_counter 
            //          << ", 期望WC=" << expected_wkc_
            //          << ", 主站状态=" << master_state.slaves_responding 
            //          << ", AL状态=0x" << std::hex << master_state.al_states << std::dec
            //          << ", 链路=" << (master_state.link_up ? "UP" : "DOWN") << std::endl;
        }
        
        // 严谨的从站状态判断 (对应ryhand6_ncb.c的成功条件)
        int al_state_ok = (master_state.al_states == 0x08);  // OP状态
        int wc_ok = (domain_state.working_counter >= expected_wkc_);  // 工作计数器正确
        int link_ok = master_state.link_up;  // 链路正常
        int slaves_ok = (master_state.slaves_responding > 0);  // 有响应的从站
        
        if (wait_cycles % 1000 == 0) { // 每1秒打印详细状态
            //std::cout << "   [DEBUG] 状态检查: AL=" << std::hex << master_state.al_states 
            //          << std::dec << " WC=" << domain_state.working_counter 
            //          << " 链路=" << (link_ok ? "UP" : "DOWN")
            //          << " 从站=" << master_state.slaves_responding << std::endl;
        }
        
        // 严格的OP状态判断：必须AL状态=0x8 AND 工作计数器=3
        if (al_state_ok && wc_ok && link_ok && slaves_ok) {
            //std::cout << "   [OK] 从站成功进入OP状态!" << std::endl;
            //std::cout << "   [OK] AL状态: 0x" << std::hex << master_state.al_states << std::dec << " (OP)" << std::endl;
            //std::cout << "   [OK] 工作计数器: " << domain_state.working_counter << std::endl;
            //std::cout << "   [OK] 链路状态: UP" << std::endl;
            break;
        }
        
        // 如果条件不满足，打印具体原因
        if (wait_cycles % 1000 == 0) {
            if (!al_state_ok) {
                //std::cout << "   [DEBUG] 等待AL状态进入OP (当前: 0x" << std::hex << master_state.al_states << std::dec << ")" << std::endl;
            }
            if (!wc_ok) {
                //std::cout << "   [DEBUG] 等待工作计数器达到期望值 (当前: " << domain_state.working_counter << ", 期望: " << expected_wkc_ << ")" << std::endl;
            }
            if (!link_ok) {
                //std::cout << "   [DEBUG] 等待链路状态恢复" << std::endl;
            }
            if (!slaves_ok) {
                //std::cout << "   [DEBUG] 等待从站响应" << std::endl;
            }
        }
        
        wait_cycles++;
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    
    if (wait_cycles >= max_wait_cycles) {
        //std::cout << "   [WARN] 等待超时，但继续运行..." << std::endl;
    }
    
    std::cout << "   [OK] 初始化完成，周期任务继续运行" << std::endl;
    //std::cout << "   [INFO] 数据交换将持续进行，请观察周期任务输出..." << std::endl;
    
    // 确保周期任务在后台持续运行
    //std::cout << "   [INFO] 周期任务状态: " << (running_ ? "运行中" : "已停止") << std::endl;
    //std::cout << "   [INFO] 周期线程状态: " << (cyclic_thread_.joinable() ? "活跃" : "非活跃") << std::endl;
    
    //std::cout << "=== EtherCAT Bridge 初始化完成 ===" << std::endl;
    initialized_ = true;
    return true;
#else
    std::cerr << "EtherCAT library not available" << std::endl;
    return false;
#endif
}

bool ethercat_bridge::start() {
    if (running_) {
        //std::cout << "EtherCAT bridge already running" << std::endl;
        return true;
    }
    
    // 周期线程已在initialize()中启动，这里只需要标记为运行状态
    std::cout << "EtherCAT bridge started" << std::endl;
    running_ = true;
    return true;
}

void ethercat_bridge::stop() {
        if (!running_) {
            return;
        }
    
        running_ = false;
    
    if (cyclic_thread_.joinable()) {
        cyclic_thread_.join();
    }
    
    std::cout << "EtherCAT bridge stopped" << std::endl;
}

bool ethercat_bridge::is_running() const {
    return running_;
}

void ethercat_bridge::set_hand_mode(hand_mode_t mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_hand_mode_ = mode;
    std::cout << "Hand mode set to: " << static_cast<int>(mode) << std::endl;
}

void ethercat_bridge::set_hand_index(int index) {
    std::lock_guard<std::mutex> lock(mutex_);
    hand_index_ = index;
}

ec_master_state_t ethercat_bridge::get_master_state() const {
#ifdef ECAT_LIB_FOUND
    ec_master_state_t state;
    if (master_) {
        ecrt_master_state(master_, &state);
    }
    return state;
#else
    return master_state_;
#endif
}

bool ethercat_bridge::scan_slaves() {
    //std::cout << "   扫描 EtherCAT 从站..." << std::endl;
    

    // 等待主站状态稳定
    //std::cout << "   等待主站状态稳定..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 检查主站状态
    //std::cout << "   检查主站状态..." << std::endl;
    ecrt_master_state(master_, &master_state_);
    
    //std::cout << "   主站状态信息:" << std::endl;
    //std::cout << "     - 响应的从站数量: " << master_state_.slaves_responding << std::endl;
    //std::cout << "     - AL 状态: 0x" << std::hex << master_state_.al_states << std::dec << std::endl;
    //std::cout << "     - 链路状态: " << (master_state_.link_up ? "UP" : "DOWN") << std::endl;
    
    if (master_state_.slaves_responding == 0) {
        std::cerr << "   [ERROR] 没有从站响应" << std::endl;
        return false;
    }
    
    //std::cout << "   [OK] 找到 " << master_state_.slaves_responding << " 个响应的从站" << std::endl;
    
    // 获取从站信息并计算期望工作计数器
    slave_count_ = master_state_.slaves_responding;
    expected_wkc_ = 0;
    
    for (int i = 0; i < slave_count_ && i < 32; i++) {
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "   [ERROR] 无法获取从站 " << i << " 信息" << std::endl;
            continue;
        }
        
        //std::cout << "   从站 " << i << " (" << slave_info.name << "): +3 WC" << std::endl;
        expected_wkc_ += 3; // ryhand从站贡献3个工作计数器
    }
    
    std::cout << "   计算得出期望工作计数器: " << expected_wkc_ << std::endl;
    
    return true;
#else
    return false;
#endif
}

bool ethercat_bridge::configure_slaves() {
    //std::cout << "   配置 EtherCAT 从站..." << std::endl;
    

    //std::cout << "   需要配置 " << slave_count_ << " 个从站" << std::endl;
    
    for (int i = 0; i < slave_count_ && i < 32; i++) {
        //std::cout << "   配置从站 " << i << "..." << std::endl;
        
        // 获取从站信息
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "     [ERROR] 无法获取从站 " << i << " 信息" << std::endl;
            continue;
        }
        
        //std::cout << "     从站 " << i << " 名称: " << slave_info.name << std::endl;
        
        // 根据从站名称配置
        if (strcmp(slave_info.name, "ryhand") == 0) {
           // std::cout << "     配置ryhand从站..." << std::endl;
            
            // 启用DC配置（与ryhand6_ncb.c一致）
            //std::cout << "     配置DC（分布式时钟）..." << std::endl;
            ecrt_slave_config_dc(slave_configs_[i], 0x0300, PERIOD_NS, PERIOD_NS*2/10, 0, 0);
            //std::cout << "     [OK] 0x0300 DC 配置完成" << std::endl;
        } else if (strcmp(slave_info.name, "XNDDrive") == 0) {
            //std::cout << "     配置XNDDrive从站..." << std::endl;
        } else if (strcmp(slave_info.name, "ESC_Switch") == 0) {
            //std::cout << "     配置ESC_Switch从站..." << std::endl;
        } else {
            //std::cout << "     未知从站类型: " << slave_info.name << std::endl;
        }
            
        // 配置从站
        slave_configs_[i] = ecrt_master_slave_config(master_, 0, i, slave_info.vendor_id, slave_info.product_code);
        if (!slave_configs_[i]) {
            std::cerr << "     [ERROR] 从站 " << i << " 配置失败" << std::endl;
            std::cerr << "     尝试继续配置其他从站..." << std::endl;
            continue;
        }
        
        //std::cout << "     [OK] 从站 " << i << " 配置成功" << std::endl;
        //std::cout << "     从站配置指针: " << slave_configs_[i] << std::endl;
    }
    
    // 检查是否有任何从站配置成功
    int configured_count = 0;
    for (int i = 0; i < slave_count_ && i < 32; i++) {
        if (slave_configs_[i]) {
            configured_count++;
        }
    }
    
    if (configured_count == 0) {
        std::cerr << "   [ERROR] 没有从站配置成功" << std::endl;
        return false;
    }
    
    //std::cout << "   [OK] " << configured_count << "/" << slave_count_ << " 个从站配置完成" << std::endl;
    return true;
#else
    return false;
#endif
}

void ethercat_bridge::cyclic_task() {
    std::cout << "EtherCAT cyclic task started" << std::endl;


    // 设置信号处理，确保程序可以正常退出 (对应ryhand6_ncb.c的signal_handler)
    signal(SIGINT, [](int sig) {
        std::cout << "\n[INFO] 收到信号 " << sig << "，正在安全退出..." << std::endl;
        exit(0);
    });
    signal(SIGTERM, [](int sig) {
        std::cout << "\n[INFO] 收到信号 " << sig << "，正在安全退出..." << std::endl;
        exit(0);
    });
    
    // 实时优先级已在主线程中设置，这里只确认
    std::cout << "Using priority " << sched_get_priority_max(SCHED_FIFO) << "." << std::endl;
    
    // 锁定内存（防止EtherCAT数据被交换到磁盘）
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    } else {
        std::cout << " [OK] 内存锁定成功" << std::endl;
    }
    
    // 设置时间基准 (对应ryhand6_ncb.c的时间管理)
    auto t0 = std::chrono::high_resolution_clock::now();
    typedef std::chrono::high_resolution_clock clock_t;
    
    int cycle_count = 0;
    constexpr int64_t expected_cycle_us = PERIOD_NS / 1000;
    
    try {
        while (running_) {
            cycle_count++;

            static auto last_cycle_time = std::chrono::high_resolution_clock::now();
            static int64_t cycle_min_us = std::numeric_limits<int64_t>::max();
            static int64_t cycle_max_us = 0;
            static int64_t cycle_total_us = 0;
            static int64_t cycle_total_jitter_us = 0;
            static int64_t cycle_max_jitter_us = 0;
            static int64_t cycle_min_jitter_us = std::numeric_limits<int64_t>::max();
            static int64_t cycle_samples = 0;
            auto now_cycle = std::chrono::high_resolution_clock::now();
            int64_t cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(now_cycle - last_cycle_time).count();
            last_cycle_time = now_cycle;
            int64_t cycle_jitter_us = cycle_us - expected_cycle_us;
            int64_t cycle_jitter_abs_us = cycle_jitter_us >= 0 ? cycle_jitter_us : -cycle_jitter_us;
            cycle_min_us = std::min(cycle_min_us, cycle_us);
            cycle_max_us = std::max(cycle_max_us, cycle_us);
            cycle_total_us += cycle_us;
            cycle_total_jitter_us += cycle_jitter_abs_us;
            cycle_max_jitter_us = std::max(cycle_max_jitter_us, cycle_jitter_abs_us);
            cycle_min_jitter_us = std::min(cycle_min_jitter_us, cycle_jitter_abs_us);
            cycle_samples++;

            //std::cout << "[循环周期] " << cycle_us << " us (" << (cycle_us / 1000.0) << " ms, 目标 " << expected_cycle_us << " us, 偏差 " << cycle_jitter_us << " us)" << std::endl;
            if (cycle_samples % 200 == 0) {
                int64_t cycle_avg_us = cycle_total_us / cycle_samples;
                int64_t cycle_avg_jitter_us = cycle_total_jitter_us / cycle_samples;
               // std::cout << "[周期统计] 平均 " << cycle_avg_us << " us, 最小 " << cycle_min_us << " us, 最大 " << cycle_max_us
               //           << " us, 偏差均值 " << cycle_avg_jitter_us << " us, 偏差峰值 " << cycle_max_jitter_us
               //           << " us, 偏差最小 " << cycle_min_jitter_us << " us" << std::endl;
            }

            // 设置应用时间 (对应ryhand6_ncb.c的时间管理)
        auto now = clock_t::now();
        uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - t0).count();
        ecrt_master_application_time(master_, now_ns);
        
        // 启用DC同步 (对应ryhand6_ncb.c的DC同步)
        // 使用与ryhand6_ncb.c相同的DC同步函数
        ecrt_master_sync_reference_clock_to(master_, now_ns);
        ecrt_master_sync_slave_clocks(master_);
        
        // 接收过程数据 (对应ryhand6_ncb.c的ecrt_master_receive)
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        
        // 检查工作计数器 (对应ryhand6_ncb.c的WC检查)
        ec_domain_state_t domain_state;
        ecrt_domain_state(domain_, &domain_state);
        
        // 检查主站状态
        ec_master_state_t master_state;
        ecrt_master_state(master_, &master_state);
        
        // 详细的状态检查 (每1000个周期打印一次)
        if (cycle_count % 1000 == 0) {
        //    std::cout << "\n=== EtherCAT RX Packet (Cycle " << cycle_count << ") ===" << std::endl;
        //    std::cout << "Master State: Slaves=" << master_state.slaves_responding 
        //              << ", AL=0x" << std::hex << master_state.al_states << std::dec
        //              << ", Link=" << (master_state.link_up ? "UP" : "DOWN")
        //              << ", WC=" << domain_state.working_counter << std::endl;
            
            // 从站状态详细分析
            if (master_state.al_states == 0x01) {
               // std::cout << "   从站状态: INIT" << std::endl;
            } else if (master_state.al_states == 0x02) {
               // std::cout << "   从站状态: PREOP" << std::endl;
            } else if (master_state.al_states == 0x04) {
                //std::cout << "   从站状态: SAFEOP" << std::endl;
            } else if (master_state.al_states == 0x08) {
                //std::cout << "   从站状态: OP" << std::endl;
                } else {
                //std::cout << "   从站状态: 未知 (0x" << std::hex << master_state.al_states << std::dec << ")" << std::endl;
            }
            
            // 状态变化检测
            static uint8_t last_al_state = 0;
            if (last_al_state != master_state.al_states) {
                //std::cout << "   [状态变化] AL状态: 0x" << std::hex << last_al_state 
                  //        << " -> 0x" << master_state.al_states << std::dec << std::endl;
                last_al_state = master_state.al_states;
            }
            
            // 显示域数据信息
            if (domain_data_) {
                size_t domain_size = ecrt_domain_size(domain_);
                //std::cout << "Domain Data (first 32 bytes): ";
                for (int i = 0; i < 32 && i < domain_size; i++) {
                //    printf("%02x ", static_cast<unsigned char>(domain_data_[i]));
                }
                std::cout << std::endl;
            } else {
                //std::cout << "Domain Data: 无效 (domain_data_为空)" << std::endl;
            }
            
            // 显示手部PDO信息
            //std::cout << "  Hand 1 PDO:" << std::endl;
            //std::cout << "    TX: F1:len=0 F2:len=0 F3:len=0 F4:len=0 F5:len=0 F6:len=0" << std::endl;
            //std::cout << "    RX: F1:len=0 F2:len=0 F3:len=0 F4:len=0 F5:len=0 F6:len=0" << std::endl;
            
            
            //std::cout << "=== ROS2 EtherCAT 周期调试信息 (周期 " << cycle_count << ") ===" << std::endl;
            //std::cout << "domain_data_ = " << static_cast<void*>(domain_data_) << std::endl;
            //std::cout << "tx_data = " << static_cast<void*>(domain_data_ + tx_offset_) 
            //          << ", rx_data = " << static_cast<void*>(domain_data_ + rx_offset_) << std::endl;
            //std::cout << "tx_offset = " << tx_offset_ << ", rx_offset = " << rx_offset_ << std::endl;
            //std::cout << "========================================" << std::endl;
            
            // 详细解释AL状态
            switch (master_state.al_states) {
                case 0x01: std::cout << " (INIT)"; break;
                case 0x02: std::cout << " (PREOP)"; break;
                case 0x04: std::cout << " (SAFEOP)"; break;
                case 0x08: std::cout << " (OP)"; break;
                default: std::cout << " (UNKNOWN)"; break;
            }
            std::cout << std::endl;
            
            std::cout << "响应的从站: " << master_state.slaves_responding << std::endl;
            //std::cout << "链路状态: " << (master_state.link_up ? "UP" : "DOWN") << std::endl;
            
            // 打印域数据内容
            if (domain_data_) {
                //std::cout << "域数据指针: " << static_cast<void*>(domain_data_) << std::endl;
                //std::cout << "域数据大小: " << ecrt_domain_size(domain_) << " 字节" << std::endl;
                //std::cout << "TX偏移: " << tx_offset_ << ", RX偏移: " << rx_offset_ << std::endl;
                
                // 打印前32字节的域数据
                //std::cout << "域数据内容 (前32字节): ";
                for (int i = 0; i < 32 && i < ecrt_domain_size(domain_); i++) {
                    //printf("%02x ", static_cast<unsigned char>(domain_data_[i]));
                }
                std::cout << std::endl;
                
                // 打印TX和RX数据
                if (tx_offset_ < ecrt_domain_size(domain_)) {
                    //std::cout << "TX数据 (偏移" << tx_offset_ << "): ";
                    for (int i = 0; i < 16 && (tx_offset_ + i) < ecrt_domain_size(domain_); i++) {
                        //printf("%02x ", static_cast<unsigned char>(domain_data_[tx_offset_ + i]));
                    }
                    std::cout << std::endl;
                }
                
                if (rx_offset_ < ecrt_domain_size(domain_)) {
                    //std::cout << "RX数据 (偏移" << rx_offset_ << "): ";
                    for (int i = 0; i < 16 && (rx_offset_ + i) < ecrt_domain_size(domain_); i++) {
                        //printf("%02x ", static_cast<unsigned char>(domain_data_[rx_offset_ + i]));
                    }
                    std::cout << std::endl;
                }
            }
            //std::cout << "================================" << std::endl;
        }
        
        // 严格的数据有效性判断：必须AL状态=0x8 AND 工作计数器=3
        int al_state_ok = (master_state.al_states == 0x08);  // OP状态
        int wc_ok = (domain_state.working_counter >= expected_wkc_);  // 工作计数器正确
        int link_ok = master_state.link_up;  // 链路状态
        int slaves_ok = (master_state.slaves_responding > 0);  // 从站响应
        int data_valid = al_state_ok && wc_ok;
        
        // 注意：ryhand6_ncb.c中不发送控制字来转换EtherCAT状态
        // 控制字是用于关节电机控制的，不是EtherCAT状态转换
        // EtherCAT状态转换由主站自动处理
        if (cycle_count > 100 && !data_valid) {
            // ryhand6_ncb.c中EtherCAT状态转换是自动的，不需要手动发送控制字
            if (cycle_count % 1000 == 0) {
                std::cout << "[DEBUG] 等待EtherCAT状态自动转换..." << std::endl;
            }
        }
        
        if (!data_valid) {
            if (cycle_count % 1000 == 0) {
                std::cout << "[DEBUG] 数据无效: AL=" << std::hex << master_state.al_states 
                          << std::dec << ", WC=" << domain_state.working_counter 
                          << ", 期望WC=" << expected_wkc_ << std::endl;
                std::cout << "[DEBUG] 状态机: " << (al_state_ok ? "AL状态OK" : "AL状态错误") 
                          << ", " << (wc_ok ? "WC状态OK" : "WC状态错误") << std::endl;
            }
        } else {
            // 成功进入OP状态！
            if (cycle_count % 1000 == 0) {
                std::cout << "\n[SUCCESS] 从站成功进入OP状态！" << std::endl;
           //     std::cout << "   AL状态: 0x" << std::hex << master_state.al_states << std::dec << " (OP)" << std::endl;
           //     std::cout << "   工作计数器: " << domain_state.working_counter << " (期望: " << expected_wkc_ << ")" << std::endl;
           //     std::cout << "   数据交换正常！" << std::endl;
            }
        }
        
        // 主动发送数据：在初始化完成后持续发送（不管是否进入OP状态）
        // 添加共享内存有效性检查
        if (domain_data_ && cycle_count > 100) { // 初始化完成后就开始发送
            // 检查共享内存是否仍然有效
            if (!shm_manager_ || !shm_manager_->is_valid() || !shared_data_) {
                if (cycle_count % 1000 == 0) {
                    std::cout << "[WARNING] 共享内存无效，跳过数据发送" << std::endl;
                }
                // 跳过数据发送，但继续运行
                // 直接跳过数据发送部分
            } else {
                // 发送手部控制数据
                uint8_t* tx_data = domain_data_ + tx_offset_;
                uint8_t* rx_data = domain_data_ + rx_offset_;
                
                // 每1000个周期打印状态信息
                if (cycle_count % 1000 == 0) {
             //       std::cout << "\n=== EtherCAT Status (Cycle " << cycle_count << ") ===" << std::endl;
               //     std::cout << "Master State: Slaves=" << master_state.slaves_responding 
               //               << ", AL=0x" << std::hex << master_state.al_states << std::dec
               //                     << ", Link=" << (master_state.link_up ? "UP" : "DOWN")
               //                     << ", WC=" << domain_state.working_counter << std::endl;
                    
                    // 显示域数据信息
                    size_t domain_size = ecrt_domain_size(domain_);
                    //std::cout << "Domain Data (first 32 bytes): ";
                    for (int i = 0; i < 32 && i < domain_size; i++) {
                      //  printf("%02x ", static_cast<unsigned char>(domain_data_[i]));
                    }
                    std::cout << std::endl;
                    
                    // 显示手部PDO信息
                    //std::cout << "  Hand 1 PDO:" << std::endl;
                    //std::cout << "    TX: F1:len=0 F2:len=0 F3:len=0 F4:len=0 F5:len=0 F6:len=0" << std::endl;
                    //std::cout << "    RX: F1:len=0 F2:len=0 F3:len=0 F4:len=0 F5:len=0 F6:len=0" << std::endl;
                    
                    // 检查是否已进入OP状态
                    int al_state_tmp = (master_state.al_states == 0x08);
                    int wc_state_tmp = (domain_state.working_counter >= expected_wkc_);
                    int link_state_tmp = master_state.link_up;
                    int slaves_state_tmp = (master_state.slaves_responding > 0);
                    
                    if (al_state_tmp && wc_state_tmp && link_state_tmp && slaves_state_tmp) {
                        //std::cout << "✅ 从站已进入OP状态，可以开始数据通信测试" << std::endl;
                    } else {
                        //std::cout << "⏳ 等待从站进入OP状态..." << std::endl;
                        //std::cout << "   AL状态: " << (al_state_tmp ? "✅" : "❌") << " (当前: 0x" << std::hex << master_state.al_states << std::dec << ")" << std::endl;
                        //std::cout << "   工作计数: " << (wc_state_tmp ? "✅" : "❌") << " (当前: " << domain_state.working_counter << "/" << expected_wkc_ << ")" << std::endl;
                        //std::cout << "   链路状态: " << (link_state_tmp ? "✅" : "❌") << std::endl;
                        //std::cout << "   从站响应: " << (slaves_state_tmp ? "✅" : "❌") << " (数量: " << master_state.slaves_responding << ")" << std::endl;
                    }
                }
                
                // 检查是否已进入OP状态，只有进入OP状态后才开始数据通信
                // 注意：al_state_ok, wc_ok, link_ok, slaves_ok 已在前面定义
                
                // 添加状态变化检测（实时监控，不依赖1000周期）
                static bool last_al_state_ok = false;
                static bool last_wc_ok = false;
                static bool last_link_ok = false;
                static bool last_slaves_ok = false;
                static bool last_motion_active = false;
                
                // 检测并记录状态变化
                if (last_al_state_ok != al_state_ok) {
                    // std::cout << "\n🔵 [AL状态变化] 周期 " << cycle_count 
                    //         << " | " << (last_al_state_ok ? "正常" : "异常") 
                    //         << " -> " << (al_state_ok ? "正常" : "异常") 
                    //       << " (当前: 0x" << std::hex << master_state.al_states << std::dec << ")" << std::endl;
                    last_al_state_ok = al_state_ok;
                }
                
                if (last_wc_ok != wc_ok) {
                    //std::cout << "\n🔵 [工作计数变化] 周期 " << cycle_count 
                    //          << " | " << (last_wc_ok ? "正常" : "异常") 
                    //          << " -> " << (wc_ok ? "正常" : "异常")
                    //          << " (当前: " << domain_state.working_counter 
                    //          << ", 期望: " << expected_wkc_ << ")" << std::endl;
                    last_wc_ok = wc_ok;
                }
                
                if (last_link_ok != link_ok) {
                    //std::cout << "\n🔵 [链路状态变化] 周期 " << cycle_count 
                    //          << " | " << (last_link_ok ? "UP" : "DOWN") 
                    //          << " -> " << (link_ok ? "UP" : "DOWN") << std::endl;
                    last_link_ok = link_ok;
                }
                
                if (last_slaves_ok != slaves_ok) {
                    //std::cout << "\n🔵 [从站响应变化] 周期 " << cycle_count 
                    //        << " | " << master_state.slaves_responding 
                    //          << " (" << (last_slaves_ok ? "响应" : "无响应") 
                    //          << " -> " << (slaves_ok ? "响应" : "无响应") << ")" << std::endl;
                    last_slaves_ok = slaves_ok;
                }
                
                // 检测运动状态变化
                bool current_motion = (al_state_ok && wc_ok && link_ok && slaves_ok);
                if (last_motion_active != current_motion) {
                    //std::cout << "\n" << (current_motion ? "🟢 [运动开始]" : "🔴 [运动停止]") 
                    //         << " 周期 " << cycle_count << std::endl;
                    last_motion_active = current_motion;
                }
                
                // 只有在OP状态时才进行正弦/固定控制
                if (al_state_ok && wc_ok && link_ok && slaves_ok) {

                    // 首次观察到稳定OP，进行预热清零（避免伺服未就绪时的空转）
                    if (!op_ready_) {
                        op_ready_ = true;
                        enable_warmup_cycles_ = 50; // 约50ms清零
                        //std::cout << "[ENABLE] OP已达标，开始预热清零50ms" << std::endl;
                    }

                    // 预热阶段：向所有电机下发0目标，稳定伺服
                    if (!servo_enabled_) {
                        if (enable_warmup_cycles_ > 0 && (cycle_count % 50 == 0)) {
                            for (int mi = 0; mi < 6; ++mi) {
                                motor_params_[mi].target_position = 0;
                                motor_params_[mi].target_speed = 0;
                                motor_params_[mi].max_current = 1000;
                            }
                            update_motor_parameters();
                            write_sine_wave_to_shared_memory();
                            enable_warmup_cycles_--;
                        }
                        if (enable_warmup_cycles_ <= 0) {
                            servo_enabled_ = true;
                          //  std::cout << "[ENABLE] 预热完成，允许运动" << std::endl;
                        }
                    } else {
                        // 使用时间戳确保固定的30ms指令周期，减少抖动
                        // 这样可以避免因为循环周期抖动导致的指令周期不稳定
                        static auto last_command_dispatch = std::chrono::steady_clock::now();
                        static bool command_initialized = false;
                        static int64_t command_min_us = std::numeric_limits<int64_t>::max();
                        static int64_t command_max_us = 0;
                        static int64_t command_total_us = 0;
                        static int64_t command_total_jitter_us = 0;
                        static int64_t command_max_jitter_us = 0;
                        static int64_t command_min_jitter_us = std::numeric_limits<int64_t>::max();
                        static int64_t command_samples = 0;
                        static const int64_t expected_command_us = 30000; // 固定30ms = 30000us
                        
                        auto command_now = std::chrono::steady_clock::now();
                        int64_t time_since_last = std::chrono::duration_cast<std::chrono::microseconds>(command_now - last_command_dispatch).count();
                        
                        // 只有当时间间隔达到30ms时才更新指令，确保周期稳定
                        if (!command_initialized || time_since_last >= expected_command_us) {
                            if (!command_initialized) {
                                last_command_dispatch = command_now;
                                command_initialized = true;
                            } else {
                                // 记录统计信息
                                int64_t command_jitter_us = time_since_last - expected_command_us;
                                int64_t command_jitter_abs_us = command_jitter_us >= 0 ? command_jitter_us : -command_jitter_us;
                                command_min_us = std::min(command_min_us, time_since_last);
                                command_max_us = std::max(command_max_us, time_since_last);
                                command_total_us += time_since_last;
                                command_total_jitter_us += command_jitter_abs_us;
                                command_max_jitter_us = std::max(command_max_jitter_us, command_jitter_abs_us);
                                command_min_jitter_us = std::min(command_min_jitter_us, command_jitter_abs_us);
                                command_samples++;
                            //    std::cout << "[指令周期] " << time_since_last << " us (目标 " << expected_command_us << " us, 偏差 " << command_jitter_us << " us)" << std::endl;
                                if (command_samples % 10 == 0) {
                                    int64_t command_avg_us = command_total_us / command_samples;
                                    int64_t command_avg_jitter_us = command_total_jitter_us / command_samples;
                              //      std::cout << "[指令统计] 平均 " << command_avg_us << " us, 最小 " << command_min_us << " us, 最大 " << command_max_us
                             //                 << " us, 偏差均值 " << command_avg_jitter_us << " us, 偏差峰值 " << command_max_jitter_us
                             //                 << " us, 偏差最小 " << command_min_jitter_us << " us" << std::endl;
                                }
                                
                                // 更新最后发送时间
                                last_command_dispatch = command_now;
                            }
                            try {
                                // 生成正弦波命令或固定模式命令
                                generate_sine_wave_commands();
                                // 更新电机参数到EtherCAT域数据
                                update_motor_parameters();
                                // 写入共享内存
                                write_sine_wave_to_shared_memory();

                               // std::cout << "\n=== EtherCAT正弦运动控制状态 (Cycle " << cycle_count << ") ===" << std::endl;
                            //std::cout << "[正弦运动] 正在控制手部进行正弦/固定运动..." << std::endl;
                                
                                // 显示发送的电机参数
                                std::cout << "发送的电机参数:" << std::endl;
                                for (int i = 0; i < 6; i++) {
                                //    std::cout << "  电机" << i << ": 位置=" << motor_params_[i].target_position 
                                //              << ", 速度=" << motor_params_[i].target_speed 
                                //              << ", 电流=" << motor_params_[i].max_current << std::endl;
                                }
                                
                                // 解析RX数据（解析所有6个手指的数据）
                                //std::cout << "\n=== 6个电机接收数据解析 ===" << std::endl;
                                for (int finger_idx = 0; finger_idx < 6; finger_idx++) {
                                    unsigned int finger_rx_offset = finger_rx_offsets_[finger_idx]; // Rxlen(2) + RxData(至少8字节)
                                    
                                    if (finger_rx_offset + 10 <= ecrt_domain_size(domain_)) {
                                        // 读取长度字段（2字节）
                                        uint16_t rx_len = *((uint16_t*)(domain_data_ + finger_rx_offset));
                                        
                                        // 读取RxData中的8字节原始数据
                                        uint8_t raw_data[8];
                                        memcpy(raw_data, domain_data_ + finger_rx_offset + 2, 8);
                                        
                                        //std::cout << "\n电机" << finger_idx << "接收数据 (8字节): ";
                                        for (int j = 0; j < 8; j++) {
                                            printf("%02x ", raw_data[j]);
                                        }
                                        std::cout << std::endl;
                                        
                                        // 解析FingerServoInfo_t（8字节数据）
                                        uint64_t data_64 = 0;
                                        for (int j = 0; j < 8; j++) {
                                            data_64 |= ((uint64_t)raw_data[j]) << (j * 8);
                                        }
                                        
                                        // 提取位域
                                        uint8_t cmd = (data_64 >> 0) & 0xFF;
                                        uint8_t status = (data_64 >> 8) & 0xFF;
                                        uint16_t position = (data_64 >> 16) & 0xFFF;
                                        int16_t velocity = (int16_t)((data_64 >> 28) & 0xFFF);
                                        int16_t current = (int16_t)((data_64 >> 40) & 0xFFF);
                                        uint16_t force = (data_64 >> 52) & 0xFFF;
                                        
                                        // 符号扩展
                                        if (velocity > 2047) velocity -= 4096;
                                        if (current > 2047) current -= 4096;
                                        
                                        //std::cout << "  电机" << finger_idx << " 接收状态:" << std::endl;
                                        //std::cout << "    命令: 0x" << std::hex << (int)cmd << std::dec 
                                        //          << (cmd == 0xa0 || cmd == 0xaa ? " (正确)" : " (⚠️异常)") << std::endl;
                                        //std::cout << "    状态: " << (int)status 
                                        //          << (status == 0 ? " (正常)" : " (⚠️异常)") << std::endl;
                                        //std::cout << "    位置: " << position << " (0-4095)" << std::endl;
                                        //std::cout << "    速度: " << velocity << " (-2048~2047)" << std::endl;
                                        //std::cout << "    电流: " << current << " (-2048~2047)" << std::endl;
                                        //std::cout << "    力: " << force << " (0-4095)" << std::endl;

                                        // 对比期望与回读（仅电机2，便于验证闭环）
                                        if (finger_idx == 2) {
                                          //  std::cout << "    期望位置 vs 回读位置: "
                                          //            << motor_params_[2].target_position << " vs " << position << std::endl;
                                        }
                                    } else {
                                        //std::cout << "\n电机" << finger_idx << ": 偏移量超出域数据范围" << std::endl;
                                    }
                                }
                                //std::cout << "================================" << std::endl;
                            } catch (const std::exception& e) {
                                std::cerr << "❌ 正弦运动控制异常: " << e.what() << std::endl;
                            }
                        }
                    }
                } else {
                    // 未进入OP状态，不进行正弦运动控制
                    
                    // 实时丢包诊断（不依赖1000周期）
                    static int last_diagnostic_count = -1;
                    if (cycle_count != last_diagnostic_count) {
                        //std::cout << "\n❌ [丢包诊断] 周期 " << cycle_count << " - 条件不满足，运动停止！" << std::endl;
                        //std::cout << "   ===== 详细状态分析 =====" << std::endl;
                        
                         // std::cout << "   1. AL状态: " << (al_state_ok ? "✅ OK" : "❌ 失败") 
                         //         << " (当前: 0x" << std::hex << master_state.al_states << std::dec 
                         //         << ", 期望: 0x08)" << std::endl;
                        
                        //std::cout << "   2. 工作计数: " << (wc_ok ? "✅ OK" : "❌ 失败") 
                        //          << " (当前: " << domain_state.working_counter 
                        //          << ", 期望: " << expected_wkc_ 
                       //           << ", 差值: " << (domain_state.working_counter - expected_wkc_) << ")" << std::endl;
                        
                       // std::cout << "   3. 链路状态: " << (master_state.link_up ? "✅ OK" : "❌ 失败") 
                       //           << " (" << (master_state.link_up ? "UP" : "DOWN") << ")" << std::endl;
                        
                       // std::cout << "   4. 从站响应: " << ((master_state.slaves_responding > 0) ? "✅ OK" : "❌ 失败") 
                       //            << " (数量: " << master_state.slaves_responding << ")" << std::endl;
                        
                        // 丢包可能性分析
                        if (!wc_ok) {
                            int wc_diff = domain_state.working_counter - expected_wkc_;
                        //    std::cout << "\n   ⚠️ [可能原因] 工作计数器异常" << std::endl;
                        //   std::cout << "      - 差异: " << wc_diff << " (负值表示丢包)" << std::endl;
                            if (wc_diff == -1) {
                        //        std::cout << "      - 判断: 可能丢包1个数据包" << std::endl;
                            } else if (wc_diff < -1) {
                        //        std::cout << "      - 判断: 可能丢包" << std::abs(wc_diff) << "个数据包" << std::endl;
                            }
                        }
                        
                        if (!link_ok) {
                        //    std::cout << "\n   ⚠️ [可能原因] 链路DOWN，检查物理连接" << std::endl;
                        }
                        
                        if (!slaves_ok) {
                        //    std::cout << "\n   ⚠️ [可能原因] 从站无响应，检查从站电源和连接" << std::endl;
                        }
                        
                        //std::cout << "   ========================" << std::endl;
                        last_diagnostic_count = cycle_count;
                    }
                }
            }
        }
        
        // 发送过程数据 (对应ryhand6_ncb.c的ecrt_domain_queue)
        // 检查EtherCAT主站是否仍然有效
        if (master_ && domain_) {
            ecrt_domain_queue(domain_);
            ecrt_master_send(master_);
        } else {
            if (cycle_count % 1000 == 0) {
             //   std::cout << "[WARNING] EtherCAT主站或域无效，跳过数据发送" << std::endl;
            }
        }
        
        // 添加错误检查
        if (cycle_count % 1000 == 0) {
            //std::cout << "[DEBUG] 周期任务正常运行，周期=" << cycle_count << std::endl;
            //std::cout << "[INFO] 数据交换持续进行中..." << std::endl;
            //std::cout << "[INFO] running_状态: " << (running_ ? "true" : "false") << std::endl;
            //std::cout << "[INFO] 线程ID: " << std::this_thread::get_id() << std::endl;
        }
        
        // 休眠1ms
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }
    } catch (const std::exception& e) {
        //std::cerr << "[ERROR] 周期任务异常: " << e.what() << std::endl;
        //std::cerr << "[ERROR] 异常类型: " << typeid(e).name() << std::endl;
        //std::cerr << "[ERROR] 周期: " << cycle_count << std::endl;
        // 继续运行，不退出
    } catch (...) {
        std::cerr << "[ERROR] 周期任务未知异常，周期: " << cycle_count << std::endl;
        // 继续运行，不退出
    }
    
    std::cout << "[INFO] 周期任务退出，running_=" << (running_ ? "true" : "false") << std::endl;
    std::cout << "[INFO] 周期任务结束" << std::endl;
#endif
}

void ethercat_bridge::update_shared_memory(SharedData_t* shared_data) {
    if (!shared_data) return;
    
    pthread_mutex_lock(&shared_data->mutex);
    
    // 更新共享内存数据
    shared_data->timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // 这里可以添加从EtherCAT域数据读取的数据
    
    pthread_mutex_unlock(&shared_data->mutex);
}

std::string ethercat_bridge::get_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
        std::string status = "EtherCAT Bridge Status:\n";
        status += "  - Initialized: " + std::string(initialized_ ? "Yes" : "No") + "\n";
        status += "  - Running: " + std::string(running_ ? "Yes" : "No") + "\n";
        status += "  - Slave Count: " + std::to_string(slave_count_) + "\n";
        status += "  - Expected WKC: " + std::to_string(expected_wkc_) + "\n";
    

    if (master_) {
        ec_master_state_t state = get_master_state();
        status += "  - Master State: 0x" + std::to_string(state.al_states) + "\n";
        status += "  - Link Up: " + std::string(state.link_up ? "Yes" : "No") + "\n";
        status += "  - Slaves Responding: " + std::to_string(state.slaves_responding) + "\n";
    }
#endif
    
    return status;
}

void ethercat_bridge::cleanup() {

    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    
    // 清理从站配置
    for (int i = 0; i < 32; i++) {
        slave_configs_[i] = nullptr;
    }
#endif
}

// 实现基类纯虚函数
CommunicationType ethercat_bridge::get_type() const {
    return CommunicationType::ETHERCAT;
}

int ethercat_bridge::get_slave_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return slave_count_;
}

bool ethercat_bridge::configure_pdos() {
    //std::cout << "   配置 PDO..." << std::endl;
    

    if (slave_count_ == 0) {
      //  std::cerr << "   [ERROR] 没有从站需要配置 PDO!" << std::endl;
        return false;
    }
    
    // 重新启用PDO配置，使用正确的同步管理器配置
    //std::cout << "   重新启用PDO配置，使用正确的同步管理器配置..." << std::endl;
    
    // 为每个从站配置PDO
    for (int i = 0; i < slave_count_ && i < 32; i++) {
        if (!slave_configs_[i]) {
            std::cerr << "   [ERROR] 从站 " << i << " 配置为空，跳过 PDO 配置" << std::endl;
            continue;
        }
        
      //  std::cout << "   为从站 " << i << " 配置 PDO..." << std::endl;
        
        // 获取从站信息
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "     [ERROR] 无法获取从站 " << i << " 信息" << std::endl;
            continue;
        }
        
        // 根据从站类型配置PDO (对应ryhand6_ncb.c的PDO配置)
        if (strcmp(slave_info.name, "ryhand") == 0) {
            std::cout << "     ryhand从站PDO配置..." << std::endl;
            
            // ryhand从站使用NULL配置，让从站使用默认PDO配置 (对应ryhand6_ncb.c的slave_syncs[0] = NULL)
            ec_sync_info_t* ryhand_syncs = NULL;
            
            // 配置PDO (对应ryhand6_ncb.c第1408行)
            int result = ecrt_slave_config_pdos(slave_configs_[i], EC_END, ryhand_syncs);
        if (result != 0) {
            std::cerr << "   [ERROR] ryhand从站 " << i << " PDO 配置失败，错误代码: " << result << std::endl;
            std::cerr << "   尝试继续配置其他从站..." << std::endl;
            continue;
        } else {
                std::cout << "   [OK] ryhand从站 " << i << " PDO 配置成功（使用NULL配置，从站默认PDO）" << std::endl;
            }
        }
    }
    
    // 注册PDO条目到域
    //std::cout << "   注册 PDO 条目到域..." << std::endl;
    
    // 使用简化的PDO条目注册
    static unsigned int off_out[32], off_in[32];
    static ec_pdo_entry_reg_t rhand_single_regs[] = {
        {0, 0, 0x00002A3F, 0x00050004, 0x7000, 1, &off_out[0]}, // Txlen_1
        {0, 0, 0x00002A3F, 0x00050004, 0x7000, 2, &off_out[1]}, // TxData_1
        {0, 0, 0x00002A3F, 0x00050004, 0x6000, 1, &off_in[0]},  // Rxlen_1
        {0, 0, 0x00002A3F, 0x00050004, 0x6000, 2, &off_in[1]},  // RxData_1
        {}
    };
    
    //std::cout << "   开始注册PDO条目到域..." << std::endl;
    int reg_result = ecrt_domain_reg_pdo_entry_list(domain_, rhand_single_regs);
    if (reg_result != 0) {
        //std::cerr << "   [ERROR] PDO 条目注册失败，错误代码: " << reg_result << std::endl;
        //std::cerr << "   可能原因: 从站不支持此PDO配置或配置不匹配" << std::endl;
        return false;
    } else {
        //std::cout << "   [OK] PDO 条目注册成功" << std::endl;
        //std::cout << "   ✓ PDO条目注册完成，偏移量信息:" << std::endl;
        //std::cout << "     输出偏移量 (0x7000系列):" << std::endl;
        //std::cout << "       0x7000,1=" << off_out[0] << " 字节" << std::endl;
        //std::cout << "       0x7000,2=" << off_out[1] << " 字节" << std::endl;
        //std::cout << "     输入偏移量 (0x6000系列):" << std::endl;
        //std::cout << "       0x6000,1=" << off_in[0] << " 字节" << std::endl;
        //std::cout << "       0x6000,2=" << off_in[1] << " 字节" << std::endl;
        
        // 设置偏移量
        tx_offset_ = off_out[0];
        rx_offset_ = off_in[0];
    }
    
    // 检查域数据大小
    size_t domain_size = ecrt_domain_size(domain_);
    //std::cout << "   域数据大小: " << domain_size << " 字节" << std::endl;
    
    //std::cout << "   [OK] PDO 配置完成（已跳过PDO配置和注册）" << std::endl;
    return true;
#else
    return false;
#endif
}

bool ethercat_bridge::register_pdo_entries() {
    //std::cout << "   注册 PDO 条目..." << std::endl;
    

    if (slave_count_ == 0) {
        std::cerr << "   [ERROR] 没有从站需要注册 PDO!" << std::endl;
        return false;
    }
    
    // 暂时跳过PDO条目注册，测试从站是否能自然进入OP状态
    //std::cout << "   暂时跳过PDO条目注册，测试从站自然状态转换..." << std::endl;
    
    // 检查域数据大小
    size_t domain_size = ecrt_domain_size(domain_);
    //std::cout << "   域数据大小: " << domain_size << " 字节" << std::endl;
    
    //std::cout << "   [OK] PDO 条目注册完成（已跳过注册）" << std::endl;
    return true;
#else
    return false;
#endif
}

bool ethercat_bridge::wait_for_op_state() {
    //std::cout << "   等待从站进入 OP 状态..." << std::endl;
    

    const int max_attempts = 1200; // 120秒
    const int delay_ms = 100;
    
    //std::cout << "   最大等待时间: " << (max_attempts * delay_ms / 1000) << " 秒" << std::endl;
    
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        // 检查主站状态
        ec_master_state_t master_state;
        ecrt_master_state(master_, &master_state);
        
        // 检查域状态
        ec_domain_state_t domain_state;
        ecrt_domain_state(domain_, &domain_state);
        
        //std::cout << "   等待中... 周期 " << attempt 
        //          << ", WC=" << domain_state.working_counter 
        //          << ", 期望WC=" << expected_wkc_
        //          << ", 主站状态=" << master_state.slaves_responding
        //          << ", AL状态=0x" << std::hex << master_state.al_states << std::dec
       //           << ", 链路=" << (master_state.link_up ? "UP" : "DOWN") << std::endl;
       // std::cout << "   域工作计数器: " << domain_state.working_counter 
       //           << ", wc_state=" << domain_state.wc_state << std::endl;
        
        // 检查从站状态
        for (int i = 0; i < slave_count_ && i < 32; ++i) {
            if (!slave_configs_[i]) continue;
            
            ec_slave_config_state_t sc_state;
            ecrt_slave_config_state(slave_configs_[i], &sc_state);
         //   std::cout << "   从站[" << i << "] state: al_state=0x" << std::hex << sc_state.al_state 
         //             << std::dec << ", online=" << sc_state.online 
         //             << ", operational=" << sc_state.operational << std::endl;
        }
        
        if (master_state.al_states == 0x08 && domain_state.working_counter >= expected_wkc_) {
        //    std::cout << "   [OK] 所有从站成功进入 OP 状态!" << std::endl;
        //    std::cout << "   [OK] 工作计数器: " << domain_state.working_counter << " (期望: " << expected_wkc_ << ")" << std::endl;
            return true;
        }
        
        if (attempt < max_attempts) {
        //    std::cout << "   从站仍在PREOP状态，继续等待..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    
    // 重新获取主站状态用于错误报告
    ec_master_state_t final_master_state;
    ecrt_master_state(master_, &final_master_state);
    
    //std::cerr << "   [ERROR] 等待 OP 状态超时" << std::endl;
    //std::cerr << "   当前 AL 状态: 0x" << std::hex << final_master_state.al_states << std::dec << std::endl;
    //std::cerr << "   响应的从站: " << final_master_state.slaves_responding << std::endl;
    //std::cerr << "   链路状态: " << (final_master_state.link_up ? "UP" : "DOWN") << std::endl;
    
    // 打印从站状态检查
    //std::cout << "\n=== 从站状态检查 ===" << std::endl;
    //std::cout << "   主站状态: 0x" << std::hex << final_master_state.al_states << std::dec << " (PREOP)" << std::endl;
    
    for (int i = 0; i < slave_count_ && i < 32; ++i) {
        if (!slave_configs_[i]) continue;
        
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) continue;
        
        ec_slave_config_state_t sc_state;
        ecrt_slave_config_state(slave_configs_[i], &sc_state);
        
    //    std::cout << "   从站 " << i << ": " << slave_info.name 
    //              << ", 状态: 0x" << std::hex << sc_state.al_state << std::dec
    //              << " (SAFEOP), 位置: " << i 
    //              << ", 制造商: 0x" << std::hex << slave_info.vendor_id << std::dec
  //                << ", 产品代码: 0x" << std::hex << slave_info.product_code << std::dec << std::endl;
    }
    
    ec_domain_state_t domain_state;
    ecrt_domain_state(domain_, &domain_state);
    //std::cout << "   域状态: WC=" << domain_state.working_counter << ", 状态=" << domain_state.wc_state << std::endl;
    //std::cout << "==================" << std::endl;
    
    return false;
#else
    return false;
#endif
}

void ethercat_bridge::set_realtime_priority() {
    //std::cout << "8. 设置实时优先级..." << std::endl;
    

    // 锁定内存（防止EtherCAT数据被交换到磁盘）
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    } else {
    //    std::cout << "   [OK] 内存锁定成功" << std::endl;
    }
    
    // 线程优先级将在cyclic_task中设置
   // std::cout << "   [OK] 实时优先级设置准备完成" << std::endl;
#endif
}

void ethercat_bridge::check_slave_states() {
    // 简化的从站状态检查
    //std::cout << "检查从站状态..." << std::endl;
    

    ec_master_info_t master_info;
    if (ecrt_master(master_, &master_info)) {
        std::cerr << "无法获取主站信息" << std::endl;
        return;
    }
    
    //std::cout << "主站状态: slaves_responding=" << master_info.slave_count 
    //          << ", link_up=" << (master_info.link_up ? "true" : "false") << std::endl;
    
    ec_master_state_t master_state;
    ecrt_master_state(master_, &master_state);
    
    //std::cout << "响应的从站数量: " << master_state.slaves_responding << std::endl;
    //std::cout << "AL 状态变化: 0x" << std::hex << master_state.al_states << std::dec;
    if (master_state.al_states == 0x01) std::cout << " (INIT)";
    else if (master_state.al_states == 0x02) std::cout << " (PREOP)";
    else if (master_state.al_states == 0x04) std::cout << " (SAFEOP)";
    else if (master_state.al_states == 0x08) std::cout << " (OP)";
            std::cout << std::endl;
    //std::cout << "链路状态: " << (master_state.link_up ? "UP" : "DOWN") << std::endl;
    
    // 检查域状态
    ec_domain_state_t ds;
    ecrt_domain_state(domain_, &ds);
    //std::cout << "域状态: working_counter=" << ds.working_counter 
    //          << ", wc_state=" << ds.wc_state << std::endl;
#endif
    //std::cout << "==================" << std::endl << std::endl;
}

void ethercat_bridge::verify_pdo_configuration() {
    // 简化的PDO配置验证
    //std::cout << "验证PDO配置..." << std::endl;
    

    ec_master_info_t master_info;
    if (ecrt_master(master_, &master_info)) {
        std::cerr << "无法获取主站信息" << std::endl;
        return;
    }
    
    //std::cout << "主站信息: slave_count=" << master_info.slave_count 
    //          << ", link_up=" << (master_info.link_up ? "true" : "false") << std::endl;
    
    // 检查每个从站的PDO配置
    for (int i = 0; i < master_info.slave_count && i < 32; i++) {
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "无法获取从站 " << i << " 信息" << std::endl;
            continue;
        }
        
    //    std::cout << "从站 " << i << ": name=" << slave_info.name 
    //              << ", vendor_id=0x" << std::hex << slave_info.vendor_id
    //              << ", product_code=0x" << slave_info.product_code << std::dec << std::endl;
    }
#endif
    //std::cout << "==================" << std::endl << std::endl;
}

void ethercat_bridge::try_state_transition() {
    // 简化的状态转换尝试
    //std::cout << "尝试状态转换..." << std::endl;
    

    ec_master_info_t master_info;
    if (ecrt_master(master_, &master_info)) {
        std::cerr << "无法获取主站信息" << std::endl;
        return;
    }
    
    //std::cout << "主站信息: slave_count=" << master_info.slave_count 
    //          << ", link_up=" << (master_info.link_up ? "true" : "false") << std::endl;
    
    // 检查每个从站的状态
    for (int i = 0; i < master_info.slave_count && i < 32; i++) {
        ec_slave_info_t slave_info;
        if (ecrt_master_get_slave(master_, i, &slave_info)) {
            std::cerr << "无法获取从站 " << i << " 信息" << std::endl;
            continue;
        }
        
        //std::cout << "从站 " << i << ": name=" << slave_info.name 
        //          << ", vendor_id=0x" << std::hex << slave_info.vendor_id
        //          << ", product_code=0x" << slave_info.product_code << std::dec << std::endl;
        
        // 尝试发送状态转换命令
        uint16_t control_word = 0x0006; // 进入OP状态
        size_t result_size = sizeof(control_word);
        uint8_t control_word_bytes[2];
        control_word_bytes[0] = control_word & 0xFF;
        control_word_bytes[1] = (control_word >> 8) & 0xFF;
        int ret = ecrt_master_sdo_download(master_, i, 0x6040, 0, 
                                          control_word_bytes, result_size, NULL);
        if (ret == 0) {
        //    std::cout << "  从站[" << i << "] 状态转换命令发送成功" << std::endl;
        } else {
        //    std::cout << "  从站[" << i << "] 状态转换命令发送失败 (错误: " << ret << ")" << std::endl;
        }
    }
#endif
    //std::cout << "==================" << std::endl << std::endl;
}

void ethercat_bridge::print_enhanced_master_state() {
    // 简化的主站状态打印
    //std::cout << "打印增强主站状态..." << std::endl;
    

    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);
    
    //std::cout << "主站状态: slaves_responding=" << ms.slaves_responding 
    //          << ", al_states=0x" << std::hex << ms.al_states << std::dec
    //          << ", link_up=" << (ms.link_up ? "true" : "false") << std::endl;
    
    // 检查域状态
    ec_domain_state_t ds;
    ecrt_domain_state(domain_, &ds);
    //std::cout << "域状态: WC=" << ds.working_counter 
    //          << ", wc_state=" << ds.wc_state << std::endl;
#endif
}


ec_sync_info_t* ethercat_bridge::get_ryhand_syncs(int slave_index) {
    // 直接返回nullptr，让系统使用默认配置
    return nullptr;
}

ec_sync_info_t* ethercat_bridge::get_xnddrive_syncs(int slave_index) {
    // 暂时返回nullptr，需要根据实际的XNDDrive配置来实现
    return nullptr;
}

ec_sync_info_t* ethercat_bridge::get_esc_switch_syncs(int slave_index) {
    // 这里应该返回与ryhand6_ncb.c中slave_syncs[i]对应的配置
    // 暂时返回nullptr，需要根据实际的ESC_Switch配置来实现
    return nullptr;
}

// 正弦波控制方法实现
void ethercat_bridge::enable_sine_wave(int mode, double frequency, double amplitude, double offset) {
    sine_mode_ = mode;
    sine_frequency_ = frequency;
    sine_amplitude_ = amplitude;
    sine_offset_ = offset;
    sine_time_ = 0.0;
    sine_wave_enabled_ = true;
    
    // 重置内部时间计数（下次调用generate_sine_wave_commands时会重置）
    // 注意：实际的sine_time_ms是在generate_sine_wave_commands中作为static变量管理的
    
    RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), 
               "正弦波控制已启用: 模式=%d, 频率=%.2fHz, 振幅=%.0f, 偏移=%.0f", 
               mode, frequency, amplitude, offset);
}
    
void ethercat_bridge::disable_sine_wave() {
    sine_wave_enabled_ = false;
    // 重置时间计数器（下次启用时会从0开始）
    sine_time_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("ethercat_bridge"), "正弦波控制已禁用");
}
    
void ethercat_bridge::generate_sine_wave_commands() {
    // 参考CAN通信的实现方式（rh6_test.cpp），使用整数tick方式计算，避免浮点精度问题
    // 由于指令更新周期是30ms，所以时间步长应该对应30ms，而不是5ms
    // 这样可以确保正弦波的时间计算与实际指令发送周期一致
    static const int command_update_interval_ms = 30;  // 指令更新间隔（与实际的30ms保持一致）
    static int sine_time_ms = 0;  // 使用整数毫秒计数，避免浮点精度问题
    static bool was_disabled = true;  // 用于检测是否从禁用状态恢复
    static auto last_update_time = std::chrono::steady_clock::now();  // 记录上次更新时间
    
    // 如果当前被禁用，设置标志以便下次启用时重置
    if (!sine_wave_enabled_) {
        was_disabled = true;
        // 自动启用正弦波（如果从站已进入OP状态）
        enable_sine_wave(0, 0.1, 1500, 1500); // 模式0, 使用默认参数
        // 启用后，下次调用时会重置时间计数
        return;
    }
    
    // 如果刚从禁用状态恢复，重置时间计数
    if (was_disabled && sine_wave_enabled_) {
        sine_time_ms = 0;
        was_disabled = false;
        last_update_time = std::chrono::steady_clock::now();
    }
    
    // 根据实际经过的时间来更新sine_time_ms，而不是固定的步长
    // 这样可以确保即使指令周期有微小抖动，正弦波的时间仍然是准确的
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time).count();
    
    // 更新时间（每次增加实际经过的时间，但确保不超过一个周期）
    sine_time_ms = (sine_time_ms + static_cast<int>(elapsed_ms)) % 100000;
    last_update_time = now;
    
    // 计算正弦波参数（与rh6_test.cpp完全一致）
    // ========== 正弦运动周期配置 ==========
    // period = 10000ms = 10秒，即完成一个完整的正弦波周期需要10秒
    // 频率 = 1 / period = 1/10 = 0.1 Hz
    // 指令更新周期 = 30ms，即每30ms发送一次新指令
    // 一个完整周期内发送的命令数 = 10000ms / 30ms ≈ 333个命令
    float period = 10000.0f;   // 周期 = 10000毫秒 = 10秒 (与rh6_test.cpp完全一致)
    float amplitude = static_cast<float>(sine_amplitude_);  // 使用配置的振幅
    
    // 使用整数tick计算，与CAN代码完全一致
    float fs = sin(2 * M_PI * sine_time_ms / period);   // 正弦值
    float fc = cos(2 * M_PI * sine_time_ms / period);   // 余弦值（用于速度计算）
    
    // 调试输出：每100次更新打印一次周期信息（约3秒打印一次）
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {
        float current_period_progress = (sine_time_ms % static_cast<int>(period)) / period * 100.0f;
    //    std::cout << "[正弦波周期信息] 总周期=" << (period/1000.0f) << "秒, " 
    //              << "当前进度=" << current_period_progress << "%, "
    //              << "已运行时间=" << (sine_time_ms/1000.0f) << "秒" << std::endl;
    }
    
    // 速度倍数：大幅降低速度确保电机有充足时间到达目标位置，实现大幅度运动
    // 如果电机跟不上，可以进一步降低此值（0.2-0.4）
    float speed_multiplier = 0.3f;  // 速度倍数（0.3 = 30%速度），确保电机能稳定跟随
    
    for (int i = 0; i < 6; i++) {
        float p1, p2;
        
        // 根据模式调整位置（与rh6_test.cpp完全一致）
        switch (sine_mode_) {
            case 0: // 原始位置控制（完全参考rh6_test.cpp的实现）
                // CAN代码公式：p1 = amplitude + amplitude * fs
                // 范围：[0, 2*amplitude]，对于amplitude=1500，范围是[0, 3000]
                // 但为了安全，限制在[500, 3500]范围内，避免到达限位（0和4095）
                p1 = amplitude + amplitude * fs;
                // 限制位置范围在[500, 3500]，确保远离限位
                p1 = std::max(500.0f, std::min(3500.0f, p1));
                
                // 速度计算：恢复CAN通信的动态速度公式
                // 动态速度可以根据位置变化自动调整，比固定速度更平滑
                // 速度倍数0.3确保电机能稳定跟上位置变化
                p2 = 1000.0f * 1000.0f * (amplitude * 4.0f) / 4095.0f * fc / period * speed_multiplier + 800.0f;
                if (i <= 1) {
                    // 前两个手指保持静止（与rh6_test.cpp完全一致）
                    motor_params_[i].target_position = 0;
                    motor_params_[i].target_speed = 1000;  // 与rh6_test.cpp一致
                } else {
                    // 确保位置值在安全范围内 [500, 3500]，避免限位
                    // 确保速度值为正数且合理
                    p2 = std::max(500.0f, std::min(1500.0f, p2)); // 限制速度在500-1500范围内
                    motor_params_[i].target_position = static_cast<uint16_t>(p1);
                    motor_params_[i].target_speed = static_cast<uint16_t>(p2);
                }
                break;
                
            case 1: // 角度控制
                p1 = 40.0f + 40.0f * fs; // 角度范围 [0, 80]
                if (i <= 1) {
                    // 电机0和1保持静止
                    motor_params_[i].target_position = 0;
                    motor_params_[i].target_speed = 0;
                } else {
                    // 角度转位置：p1 (0-80度) -> (0-4095)
                    p1 = p1 * 4095.0f / 180.0f; // 使用浮点数除法避免精度损失
                    // 确保位置值在有效范围内 [0, 4095]
                    p1 = std::max(0.0f, std::min(4095.0f, p1));
                    motor_params_[i].target_position = static_cast<uint16_t>(p1);
                    motor_params_[i].target_speed = 1000;
                }
                break;
                
            case 2: // 末端位置控制
                p1 = 50.0f + 30.0f * fs; // 位置范围 [20, 80]
                if (i <= 1) {
                    // 电机0和1保持静止
                    motor_params_[i].target_position = 0;
                    motor_params_[i].target_speed = 0;
                } else {
                    // 位置转关节：p1 (20-80) -> (0-4095)
                    p1 = p1 * 4095.0f / 100.0f; // 使用浮点数除法避免精度损失
                    // 确保位置值在有效范围内 [0, 4095]
                    p1 = std::max(0.0f, std::min(4095.0f, p1));
                    motor_params_[i].target_position = static_cast<uint16_t>(p1);
                    motor_params_[i].target_speed = 1000;
                }
                break;
            case 3: // 固定位置控制（只让电机2动）
                // 只有电机2设置固定位置，其他保持静止
                if (i == 2) {
                    motor_params_[i].target_position = 2048;  // 固定位置2048
                    motor_params_[i].target_speed = 1000;
                } else {
                    // 其他电机保持静止
                    motor_params_[i].target_position = 0;
                    motor_params_[i].target_speed = 0;
                }
                break;
                
            default:
                // 未知模式，所有电机保持静止
                motor_params_[i].target_position = 0;
                motor_params_[i].target_speed = 0;
                if (i == 0) {  // 只在第一个电机时打印一次警告
                    RCLCPP_WARN(rclcpp::get_logger("ethercat_bridge"), 
                               "未知的正弦波模式: %d，所有电机保持静止", sine_mode_);
                }
                break;
        }
        
        // 设置电机参数
        motor_params_[i].max_current = 1000;     // 最大电流
        motor_params_[i].command = 0xee;         // 位置控制命令
    }
}

void ethercat_bridge::update_motor_parameters() {
    // 发送正弦运动命令到EtherCAT从站
    // 在一个PDO通道中发送所有6个手指的数据（共42字节：6个手指 x 7字节）
    
    // 准备6个手指的数据（TX命令字使用0xAA，格式需与CAN一致）
    ServoData servo_data_array[6];
    for (int i = 0; i < 6; i++) {
        // 清零结构体，确保未使用的字节为0
        memset(&servo_data_array[i], 0, sizeof(ServoData));
        
        // 设置命令字节
        servo_data_array[i].pucDat[0] = 0xaa;
        // 设置位置、速度、电流（小端序，由编译器自动处理）
        servo_data_array[i].stuCmd.cmd = 0xaa;
        servo_data_array[i].stuCmd.usPos = static_cast<uint16_t>(motor_params_[i].target_position);
        servo_data_array[i].stuCmd.usSpd = static_cast<uint16_t>(motor_params_[i].target_speed);
        servo_data_array[i].stuCmd.usMaxCur = static_cast<uint16_t>(motor_params_[i].max_current);
        servo_data_array[i].stuCmd.res = 0;
        
        // 验证编码正确性（仅电机2）
        if (i == 2) {
            // 验证位置值是否正确编码
            uint16_t expected_pos = motor_params_[i].target_position;
            uint16_t encoded_pos = servo_data_array[i].stuCmd.usPos;
            if (expected_pos != encoded_pos) {
                std::cerr << "[ERROR] Motor2位置编码错误: 期望=" << expected_pos 
                          << ", 编码后=" << encoded_pos << std::endl;
            }
        }
    }
    
    // 写入EtherCAT域数据 - 发送6个手指的数据
    for (int i = 0; i < 6; i++) {
        // 每个手指的Txlen偏移和TxData偏移（差16位=2字节）
        unsigned int txlen_offset = finger_tx_offsets_[i];
        unsigned int txdata_offset = txlen_offset + 2;
        
        if (txdata_offset + 25 <= ecrt_domain_size(domain_)) {
            // 写入长度（7字节）
            *((uint16_t*)(domain_data_ + txlen_offset)) = 7;
            // 将手指数据写入TxData区域（24字节缓冲区）
            memcpy(domain_data_ + txdata_offset, &servo_data_array[i], 7);

            /
            if (i == 2) {
          //      std::cout << "[TX RAW] Motor2 bytes: ";
                for (int b = 0; b < 7; ++b) {
        //            printf("%02x ", static_cast<unsigned char>(*(domain_data_ + txdata_offset + b)));
                }
                std::cout << std::endl;
                
                // 验证位置编码：期望位置应该在字节1-2（小端序）
                uint16_t pos_from_bytes = *((uint16_t*)(domain_data_ + txdata_offset + 1));
                uint16_t expected_pos = motor_params_[i].target_position;
            //    std::cout << "[TX VERIFY] Motor2位置验证: 期望=" << expected_pos 
            //              << ", 编码=" << pos_from_bytes 
            //              << (expected_pos == pos_from_bytes ? " ✓" : " ✗编码错误!") << std::endl;
            }
        }
    }
    
    // 调试输出：显示发送的数据（限频，主循环已有与下发一致的打印）
    static int debug_count = 0;
    if (++debug_count % 1000 == 0) {
        //std::cout << "\n=== EtherCAT TX数据发送（6个手指）===" << std::endl;
        std::cout << "当前模式: " << sine_mode_ << std::endl;
        for (int i = 0; i < 6; i++) {
        //    std::cout << "  手指" << i << ": 位置=" << motor_params_[i].target_position 
         //             << ", 速度=" << motor_params_[i].target_speed << std::endl;
        }
    }
    
    // 同时将电机参数写入共享内存（用于ROS接口）
    if (shared_data_) {
        pthread_mutex_lock(&shared_data_->mutex);
        
        // 将电机参数写入共享内存（使用ServoData格式，包含命令字节）
        for (int i = 0; i < 6; i++) {
            ServoData servo_data;
            servo_data.pucDat[0] = 0xaa;  // 命令字节
            servo_data.stuCmd.usPos = motor_params_[i].target_position;
            servo_data.stuCmd.usSpd = motor_params_[i].target_speed;
            servo_data.stuCmd.usMaxCur = motor_params_[i].max_current;
            
            // 写入共享内存（使用7字节，与CAN通信一致）
            shared_data_->ryhand[0].tx_len[i] = 7;
            memcpy(shared_data_->ryhand[0].tx_data[i], &servo_data, 7);
        }
        
        // 更新标志
        shared_data_->tx_data_cnt++;
        shared_data_->tx_nh = 6;
        
        pthread_mutex_unlock(&shared_data_->mutex);
    }
}

void ethercat_bridge::write_sine_wave_to_shared_memory() {
    if (!shared_data_) {
        return;
    }
    
    pthread_mutex_lock(&shared_data_->mutex);
    
    // 将电机参数写入共享内存（使用ServoData格式，包含命令字节）
    for (int i = 0; i < 6; i++) {
        ServoData servo_data;
        servo_data.pucDat[0] = 0xaa;  // 命令字节
        servo_data.stuCmd.usPos = motor_params_[i].target_position;
        servo_data.stuCmd.usSpd = motor_params_[i].target_speed;
        servo_data.stuCmd.usMaxCur = motor_params_[i].max_current;
        
        // 写入共享内存（使用7字节，与CAN通信一致）
        shared_data_->ryhand[0].tx_len[i] = 7;
        memcpy(shared_data_->ryhand[0].tx_data[i], &servo_data, 7);
    }
    
    // 更新标志
    shared_data_->tx_data_cnt++;
    shared_data_->tx_nh = 6;
    
    pthread_mutex_unlock(&shared_data_->mutex);
}

} // namespace ruiyan::rh6
