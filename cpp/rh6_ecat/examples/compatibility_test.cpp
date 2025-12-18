#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

// C 兼容头文件
extern "C" {
#include "rh6_ecat/shm_utils.h"
}

// C++ 兼容头文件
#include "rh6_ecat/shm_utils_cpp.hpp"

using namespace ruiyan::rh6;

int main() {
    std::cout << "=== C/C++ 兼容性测试 ===" << std::endl;
    
    try {
        // 测试 C++ 兼容层
        std::cout << "1. 测试 C++ 兼容层..." << std::endl;
        auto shm_manager = SharedMemoryUtils::create_manager(true, "/test_ethercat_data");
        if (!shm_manager || !shm_manager->is_valid()) {
            std::cerr << "C++ 兼容层测试失败" << std::endl;
            return -1;
        }
        std::cout << "   ✓ C++ 兼容层工作正常" << std::endl;
        
        // 测试 C 语言函数
        std::cout << "2. 测试 C 语言函数..." << std::endl;
        SharedData_t* c_shared_data = create_shared_memory(0); // 打开已存在的
        if (!c_shared_data) {
            std::cerr << "C 语言函数测试失败" << std::endl;
            return -1;
        }
        std::cout << "   ✓ C 语言函数工作正常" << std::endl;
        
        // 测试数据访问
        std::cout << "3. 测试数据访问..." << std::endl;
        SharedData_t* cpp_data = shm_manager->get();
        
        // 通过 C++ 写入数据
        cpp_data->timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        cpp_data->master_status.slaves_responding = 6;
        cpp_data->master_status.link_up = 1;
        cpp_data->data_valid = 1;
        
        // 通过 C 语言读取数据
        SharedData_t local_copy;
        if (read_shared_data(c_shared_data, &local_copy) == 0) {
            if (local_copy.timestamp_ns == cpp_data->timestamp_ns &&
                local_copy.master_status.slaves_responding == 6 &&
                local_copy.master_status.link_up == 1) {
                std::cout << "   ✓ 数据访问兼容性正常" << std::endl;
            } else {
                std::cerr << "   ✗ 数据访问兼容性失败" << std::endl;
                return -1;
            }
        } else {
            std::cerr << "   ✗ 数据读取失败" << std::endl;
            return -1;
        }
        
        // 测试结构体大小
        std::cout << "4. 测试结构体大小..." << std::endl;
        size_t cpp_size = sizeof(SharedData_t);
        size_t c_size = sizeof(SharedData_t);
        if (cpp_size == c_size) {
            std::cout << "   ✓ 结构体大小一致: " << cpp_size << " 字节" << std::endl;
        } else {
            std::cerr << "   ✗ 结构体大小不一致: C++=" << cpp_size << ", C=" << c_size << std::endl;
            return -1;
        }
        
        // 测试内存对齐
        std::cout << "5. 测试内存对齐..." << std::endl;
        if (reinterpret_cast<uintptr_t>(cpp_data) % alignof(SharedData_t) == 0) {
            std::cout << "   ✓ 内存对齐正常" << std::endl;
        } else {
            std::cerr << "   ✗ 内存对齐异常" << std::endl;
            return -1;
        }
        
        // 清理
        destroy_shared_memory(c_shared_data);
        shm_manager.reset();
        
        std::cout << "=== 所有测试通过！C/C++ 兼容性正常 ===" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "测试失败: " << e.what() << std::endl;
        return -1;
    }
}
