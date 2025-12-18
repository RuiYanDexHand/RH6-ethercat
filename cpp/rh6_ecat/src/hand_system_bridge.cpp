#include <iostream>
#include <signal.h>
#include <memory>
#include <chrono>
#include <thread>
#include "rh6_ecat/communication_bridge.hpp"
#include "rh6_ecat/ros_hand_interface.hpp"
#include <rclcpp/rclcpp.hpp>

// 全局变量
static volatile int run = 1;
static std::unique_ptr<ruiyan::rh6::CommunicationBridge> comm_bridge = nullptr;
static std::unique_ptr<ruiyan::rh6::RosHandInterface> ros_interface = nullptr;

// 信号处理函数
void signal_handler(int sig) {
    std::cout << "\n[INFO] 收到信号 " << sig << "，准备安全退出..." << std::endl;
    std::cout << "[INFO] 正在停止周期任务..." << std::endl;
    std::cout << "[INFO] 设置程序退出标志..." << std::endl;
    std::cout << "[INFO] 用户主动退出程序" << std::endl;
    run = 0;
}

int main(int argc, char **argv) {
    std::cout << "Starting RH6 Hand System Bridge..." << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "信号处理已设置: SIGINT, SIGTERM" << std::endl;
    std::cout << "程序将只响应用户的Ctrl+C信号" << std::endl;
    
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    
    std::cout << "ROS2初始化完成" << std::endl;
    std::cout << "ROS2状态: " << (rclcpp::ok() ? "正常" : "异常") << std::endl;
    
    try {
        // 解析命令行参数
        std::string comm_type = ""; // 不设置默认值，要求用户明确选择
        std::string comm_config = "";
        
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--comm" && i + 1 < argc) {
                comm_type = argv[++i];
            } else if (arg == "--config" && i + 1 < argc) {
                comm_config = argv[++i];
            } else if (arg == "--help") {
                std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
                std::cout << "Options:" << std::endl;
                std::cout << "  --comm <type>     Communication type (REQUIRED: ethercat, can, serial, tcp, udp, shared_memory)" << std::endl;
                std::cout << "  --config <file>   Configuration file path" << std::endl;
                std::cout << "  --help            Show this help message" << std::endl;
                std::cout << "Supported communication types:" << std::endl;
                auto supported_types = ruiyan::rh6::CommunicationBridgeFactory::get_supported_types();
                for (const auto& type : supported_types) {
                    std::cout << "  - " << type << std::endl;
                }
                std::cout << std::endl;
                std::cout << "Example usage:" << std::endl;
                std::cout << "  " << argv[0] << " --comm ethercat" << std::endl;
                std::cout << "  " << argv[0] << " --comm can" << std::endl;
                std::cout << "  " << argv[0] << " --comm serial" << std::endl;
                return 0;
            }
        }
        
        // 检查是否指定了通信方式
        if (comm_type.empty()) {
            std::cerr << "Error: Communication type is required!" << std::endl;
            std::cerr << "Please specify --comm <type> parameter." << std::endl;
            std::cerr << "Use --help for more information." << std::endl;
            return 1;
        }
        
        // 创建通信桥梁
        std::cout << "Creating " << comm_type << " communication bridge..." << std::endl;
        comm_bridge = ruiyan::rh6::CommunicationBridgeFactory::create(comm_type);
        
        if (!comm_bridge) {
            std::cerr << "Failed to create communication bridge for type: " << comm_type << std::endl;
            std::cerr << "Supported types: ";
            auto supported_types = ruiyan::rh6::CommunicationBridgeFactory::get_supported_types();
            for (const auto& type : supported_types) {
                std::cerr << type << " ";
            }
            std::cerr << std::endl;
            return -1;
        }
        
        // 初始化通信桥梁
        if (!comm_bridge->initialize(comm_config)) {
            std::cerr << "Failed to initialize communication bridge" << std::endl;
            return -1;
        }
        
        // 创建 ROS 接口
        std::cout << "Creating ROS Hand Interface..." << std::endl;
        ros_interface = std::make_unique<ruiyan::rh6::RosHandInterface>();
        
        // 初始化 ROS 接口
        if (!ros_interface->initialize()) {
            std::cerr << "Failed to initialize ROS Hand Interface" << std::endl;
            return -1;
        }
        
        // 启动通信桥梁
        std::cout << "Starting communication bridge..." << std::endl;
        if (!comm_bridge->start()) {
            std::cerr << "Failed to start communication bridge" << std::endl;
            return -1;
        }
        
        // 启动 ROS 接口
        std::cout << "Starting ROS Hand Interface..." << std::endl;
        ros_interface->start();
        
        std::cout << "RH6 Hand System Bridge started successfully!" << std::endl;
        std::cout << "Communication type: " << comm_type << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        // 主循环
        std::cout << "进入主循环，开始持续运行..." << std::endl;
        std::cout << "主循环状态: run=" << run << ", rclcpp::ok()=" << rclcpp::ok() << std::endl;
        std::cout << "程序将持续运行，直到用户按Ctrl+C..." << std::endl;
        
        int loop_count = 0;
        int consecutive_errors = 0;
        const int max_consecutive_errors = 10;
        
        while (run && rclcpp::ok()) {
            try {
                rclcpp::spin_some(ros_interface->get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                
                // 重置错误计数
                consecutive_errors = 0;
                
                // 每1000次循环打印一次状态
                if (++loop_count % 1000 == 0) {
                    std::cout << "[DEBUG] 主循环运行中，循环次数=" << loop_count 
                              << ", run=" << run << ", rclcpp::ok()=" << rclcpp::ok() << std::endl;
                    std::cout << "[DEBUG] 程序状态正常，等待用户Ctrl+C..." << std::endl;
                }
            } catch (const std::exception& e) {
                consecutive_errors++;
                std::cerr << "主循环异常: " << e.what() << std::endl;
                std::cerr << "异常类型: " << typeid(e).name() << std::endl;
                std::cerr << "连续错误次数: " << consecutive_errors << "/" << max_consecutive_errors << std::endl;
                
                if (consecutive_errors >= max_consecutive_errors) {
                    std::cerr << "连续错误过多，但程序继续运行..." << std::endl;
                    consecutive_errors = 0; // 重置计数
                }
                
                // 不退出，继续运行
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        std::cout << "主循环退出: run=" << run << ", rclcpp::ok()=" << rclcpp::ok() << std::endl;
        
        // 分析退出原因
        if (!run) {
            std::cout << "程序退出原因: 用户信号 (Ctrl+C)" << std::endl;
        } else if (!rclcpp::ok()) {
            std::cout << "程序退出原因: ROS2系统异常" << std::endl;
        } else {
            std::cout << "程序退出原因: 未知" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    // 清理
    std::cout << "Shutting down RH6 Hand System Bridge..." << std::endl;
    std::cout << "程序正常退出，开始清理资源..." << std::endl;
    
    // 设置关闭标志
    if (comm_bridge) {
        std::cout << "设置程序关闭标志..." << std::endl;
        // 这里可以设置共享内存中的关闭标志
    }
    
    // 先停止ROS2接口
    if (ros_interface) {
        std::cout << "Stopping ROS Hand Interface..." << std::endl;
        ros_interface->stop();
        ros_interface.reset();
        std::cout << "ROS Hand Interface stopped" << std::endl;
    }
    
    // 等待一段时间确保ROS接口完全停止
    std::cout << "等待ROS接口完全停止..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // 再停止通信桥
    if (comm_bridge) {
        std::cout << "Stopping communication bridge..." << std::endl;
        comm_bridge->stop();
        comm_bridge.reset();
        std::cout << "Communication bridge stopped" << std::endl;
    }
    
    // 等待一段时间确保通信桥完全停止
    std::cout << "等待通信桥完全停止..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    rclcpp::shutdown();
    
    std::cout << "RH6 Hand System Bridge stopped" << std::endl;
    return 0;
}
