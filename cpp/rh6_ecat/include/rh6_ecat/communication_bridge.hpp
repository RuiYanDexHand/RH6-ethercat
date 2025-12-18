#pragma once

#include <memory>
#include <string>
#include <functional>
#include "rh6_ecat/ryhand_types.hpp"
#include "rh6_ecat/shared_data.h"
#include "rh6_ecat/ethercat_slave_config.h"

namespace ruiyan::rh6 {

/**
 * @brief 通信方式枚举
 */
enum class CommunicationType {
    ETHERCAT = 0,    // EtherCAT 通信
    CAN = 1,         // CAN 通信
    SERIAL = 2,      // 串口通信
    TCP = 3,         // TCP 网络通信
    UDP = 4,         // UDP 网络通信
    SHARED_MEMORY = 5 // 共享内存通信
};

/**
 * @brief 通信桥梁基类
 * 
 * 定义了所有通信方式的统一接口
 */
class CommunicationBridge {
public:
    virtual ~CommunicationBridge() = default;

    /**
     * @brief 初始化通信桥梁
     * @param config 配置参数
     * @return 成功返回 true，失败返回 false
     */
    virtual bool initialize(const std::string& config) = 0;

    /**
     * @brief 启动通信桥梁
     * @return 成功返回 true，失败返回 false
     */
    virtual bool start() = 0;

    /**
     * @brief 停止通信桥梁
     */
    virtual void stop() = 0;

    /**
     * @brief 检查桥梁是否正在运行
     * @return 运行中返回 true，否则返回 false
     */
    virtual bool is_running() const = 0;

    /**
     * @brief 获取通信类型
     * @return 通信类型
     */
    virtual CommunicationType get_type() const = 0;

    /**
     * @brief 获取从站数量
     * @return 从站数量
     */
    virtual int get_slave_count() const = 0;

    /**
     * @brief 设置手部模式
     * @param mode 手部模式
     */
    virtual void set_hand_mode(hand_mode_t mode) = 0;

    /**
     * @brief 设置手部索引
     * @param index 手部索引（0=左手，1=右手）
     */
    virtual void set_hand_index(int index) = 0;

    /**
     * @brief 更新共享内存数据
     * @param shared_data 共享内存数据指针
     */
    virtual void update_shared_memory(SharedData_t* shared_data) = 0;

    /**
     * @brief 获取状态信息
     * @return 状态信息字符串
     */
    virtual std::string get_status() const = 0;
};

/**
 * @brief 通信桥梁工厂类
 */
class CommunicationBridgeFactory {
public:
    /**
     * @brief 创建通信桥梁
     * @param type 通信类型
     * @return 通信桥梁智能指针
     */
    static std::unique_ptr<CommunicationBridge> create(CommunicationType type);

    /**
     * @brief 从字符串创建通信桥梁
     * @param type_str 通信类型字符串
     * @return 通信桥梁智能指针
     */
    static std::unique_ptr<CommunicationBridge> create(const std::string& type_str);

    /**
     * @brief 获取支持的通信类型列表
     * @return 通信类型字符串列表
     */
    static std::vector<std::string> get_supported_types();

    /**
     * @brief 检查是否支持指定的通信类型
     * @param type 通信类型
     * @return 支持返回 true，否则返回 false
     */
    static bool is_supported(CommunicationType type);

    /**
     * @brief 检查是否支持指定的通信类型字符串
     * @param type_str 通信类型字符串
     * @return 支持返回 true，否则返回 false
     */
    static bool is_supported(const std::string& type_str);
};

} // namespace ruiyan::rh6

