#include "rh6_ecat/communication_bridge.hpp"
#include "rh6_ecat/ethercat_bridge.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cctype>

namespace ruiyan::rh6 {

// 前向声明其他通信桥梁类
class CanBridge;
class SerialBridge;
class TcpBridge;
class UdpBridge;
class SharedMemoryBridge;

std::unique_ptr<CommunicationBridge> CommunicationBridgeFactory::create(CommunicationType type) {
    switch (type) {
        case CommunicationType::ETHERCAT:
#ifdef ECAT_LIB_FOUND
            return std::make_unique<ethercat_bridge>();
#else
            std::cerr << "EtherCAT bridge not available: EtherCAT library not found" << std::endl;
            std::cerr << "Please install libethercat to enable EtherCAT communication" << std::endl;
            return nullptr;
#endif
        case CommunicationType::CAN:
            // return std::make_unique<CanBridge>();
            std::cerr << "CAN bridge not implemented yet" << std::endl;
            return nullptr;
        case CommunicationType::SERIAL:
            // return std::make_unique<SerialBridge>();
            std::cerr << "Serial bridge not implemented yet" << std::endl;
            return nullptr;
        case CommunicationType::TCP:
            // return std::make_unique<TcpBridge>();
            std::cerr << "TCP bridge not implemented yet" << std::endl;
            return nullptr;
        case CommunicationType::UDP:
            // return std::make_unique<UdpBridge>();
            std::cerr << "UDP bridge not implemented yet" << std::endl;
            return nullptr;
        case CommunicationType::SHARED_MEMORY:
            // return std::make_unique<SharedMemoryBridge>();
            std::cerr << "Shared memory bridge not implemented yet" << std::endl;
            return nullptr;
        default:
            std::cerr << "Unknown communication type: " << static_cast<int>(type) << std::endl;
            return nullptr;
    }
}

std::unique_ptr<CommunicationBridge> CommunicationBridgeFactory::create(const std::string& type_str) {
    // 转换为小写
    std::string lower_type = type_str;
    std::transform(lower_type.begin(), lower_type.end(), lower_type.begin(), ::tolower);
    
    if (lower_type == "ethercat" || lower_type == "ecat") {
        return create(CommunicationType::ETHERCAT);
    } else if (lower_type == "can") {
        return create(CommunicationType::CAN);
    } else if (lower_type == "serial" || lower_type == "rs232" || lower_type == "rs485") {
        return create(CommunicationType::SERIAL);
    } else if (lower_type == "tcp") {
        return create(CommunicationType::TCP);
    } else if (lower_type == "udp") {
        return create(CommunicationType::UDP);
    } else if (lower_type == "shm" || lower_type == "shared_memory") {
        return create(CommunicationType::SHARED_MEMORY);
    } else {
        std::cerr << "Unknown communication type string: " << type_str << std::endl;
        return nullptr;
    }
}

std::vector<std::string> CommunicationBridgeFactory::get_supported_types() {
    return {
        "ethercat",
        "can", 
        "serial",
        "tcp",
        "udp",
        "shared_memory"
    };
}


} // namespace ruiyan::rh6

