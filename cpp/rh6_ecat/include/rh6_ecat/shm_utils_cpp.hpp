#ifndef SHM_UTILS_CPP_HPP
#define SHM_UTILS_CPP_HPP

#include <memory>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>
#include <chrono>

// C 兼容头文件
extern "C" {
#include "shm_utils.h"
}

namespace ruiyan::rh6 {

/**
 * @brief C++ 共享内存管理类
 * 
 * 提供 C++ 风格的共享内存接口，内部使用 C 语言实现
 * 确保与现有的 C 语言 EtherCAT 代码完全兼容
 */
class SharedMemoryManager {
public:
    /**
     * @brief 构造函数
     * @param create_new 是否创建新的共享内存对象
     * @param shm_name 共享内存名称，默认为 "/ethercat_data"
     */
    explicit SharedMemoryManager(bool create_new = false, 
                                const std::string& shm_name = "/ethercat_data")
        : shared_data_(nullptr), shm_name_(shm_name) {
        
        // 调用 C 语言函数
        shared_data_ = create_shared_memory(create_new ? 1 : 0);
        if (!shared_data_) {
            throw std::runtime_error("Failed to create/open shared memory: " + shm_name_);
        }
    }
    
    /**
     * @brief 析构函数 - 只在程序真正结束时销毁共享内存
     */
    ~SharedMemoryManager() {
        if (shared_data_) {
            // 检查是否在程序退出时销毁
            std::cout << "SharedMemoryManager: 析构函数被调用" << std::endl;
            std::cout << "SharedMemoryManager: 检查程序状态..." << std::endl;
            
            // 检查是否有其他进程在使用共享内存
            if (shared_data_->shutdown_flag == 0) {
                std::cout << "SharedMemoryManager: 程序仍在运行，延迟销毁共享内存..." << std::endl;
                std::cout << "SharedMemoryManager: 等待20秒确保所有进程完成..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(20000)); // 增加到20秒
            }
            
            std::cout << "SharedMemoryManager: 开始销毁共享内存..." << std::endl;
            destroy_shared_memory(shared_data_);
            shared_data_ = nullptr;
            std::cout << "SharedMemoryManager: 共享内存已销毁" << std::endl;
        }
    }
    
    // 禁止拷贝构造和赋值
    SharedMemoryManager(const SharedMemoryManager&) = delete;
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;
    
    // 允许移动构造和赋值
    SharedMemoryManager(SharedMemoryManager&& other) noexcept
        : shared_data_(other.shared_data_), shm_name_(std::move(other.shm_name_)) {
        other.shared_data_ = nullptr;
    }
    
    SharedMemoryManager& operator=(SharedMemoryManager&& other) noexcept {
        if (this != &other) {
            if (shared_data_) {
                destroy_shared_memory(shared_data_);
            }
            shared_data_ = other.shared_data_;
            shm_name_ = std::move(other.shm_name_);
            other.shared_data_ = nullptr;
        }
        return *this;
    }
    
    /**
     * @brief 获取共享数据指针
     * @return SharedData_t* 共享数据指针
     */
    SharedData_t* get() const {
        return shared_data_;
    }
    
    /**
     * @brief 检查共享内存是否有效
     * @return bool 是否有效
     */
    bool is_valid() const {
        return shared_data_ != nullptr;
    }
    
    /**
     * @brief 读取共享数据
     * @param local_copy 本地数据副本
     * @return bool 是否读取成功
     */
    bool read_data(SharedData_t& local_copy) const {
        if (!shared_data_) {
            return false;
        }
        return read_shared_data(shared_data_, &local_copy) == 0;
    }
    
    /**
     * @brief 写入共享数据
     * @param local_data 本地数据
     * @return bool 是否写入成功
     */
    bool write_data(const SharedData_t& local_data) const {
        if (!shared_data_) {
            return false;
        }
        return write_shared_data(shared_data_, &local_data) == 0;
    }
    
    /**
     * @brief 获取共享内存名称
     * @return const std::string& 共享内存名称
     */
    const std::string& get_name() const {
        return shm_name_;
    }

private:
    SharedData_t* shared_data_;
    std::string shm_name_;
};

/**
 * @brief 共享内存工具函数
 */
class SharedMemoryUtils {
public:
    /**
     * @brief 创建共享内存管理器
     * @param create_new 是否创建新的共享内存
     * @param shm_name 共享内存名称
     * @return std::unique_ptr<SharedMemoryManager> 共享内存管理器
     */
    static std::unique_ptr<SharedMemoryManager> create_manager(
        bool create_new = false, 
        const std::string& shm_name = "/ethercat_data") {
        
        return std::make_unique<SharedMemoryManager>(create_new, shm_name);
    }
    
    /**
     * @brief 检查共享内存是否存在
     * @param shm_name 共享内存名称
     * @return bool 是否存在
     */
    static bool exists(const std::string& shm_name = "/ethercat_data") {
        SharedData_t* test_data = create_shared_memory(0);
        if (test_data) {
            destroy_shared_memory(test_data);
            return true;
        }
        return false;
    }
    
    /**
     * @brief 清理共享内存
     * @param shm_name 共享内存名称
     * @return bool 是否清理成功
     */
    static bool cleanup(const std::string& shm_name = "/ethercat_data") {
        // 这里可以添加清理逻辑
        return true;
    }
};

} // namespace ruiyan::rh6

#endif // SHM_UTILS_CPP_HPP
