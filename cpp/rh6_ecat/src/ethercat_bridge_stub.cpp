#include "rh6_ecat/ethercat_bridge.hpp"
#include "rh6_ecat/ethercat_slave_config.h"
#include <iostream>

namespace ruiyan::rh6 {

// EtherCAT 桥梁存根实现 - 用于在没有 EtherCAT 库时编译

ethercat_bridge::ethercat_bridge() {
    std::cout << "EtherCAT Bridge stub initialized (EtherCAT library not available)" << std::endl;
}

ethercat_bridge::~ethercat_bridge() {
    std::cout << "EtherCAT Bridge stub destroyed" << std::endl;
}

bool ethercat_bridge::initialize(const std::string& config) {
    std::cerr << "Error: EtherCAT library not available. Cannot initialize EtherCAT bridge." << std::endl;
    std::cerr << "Please install libethercat to enable EtherCAT communication." << std::endl;
    return false;
}

bool ethercat_bridge::start() {
    std::cerr << "Error: EtherCAT library not available. Cannot start EtherCAT bridge." << std::endl;
    return false;
}

void ethercat_bridge::stop() {
    std::cout << "EtherCAT Bridge stub stopped" << std::endl;
}

bool ethercat_bridge::is_running() const {
    return false;
}

void ethercat_bridge::set_hand_mode(hand_mode_t mode) {
    std::cerr << "Error: EtherCAT library not available. Cannot set hand mode." << std::endl;
}

void ethercat_bridge::set_hand_index(int index) {
    std::cerr << "Error: EtherCAT library not available. Cannot set hand index." << std::endl;
}

std::string ethercat_bridge::get_status() const {
    return "EtherCAT Bridge Status:\n  Type: EtherCAT (STUB - Library not available)\n  State: Not initialized\n  Error: EtherCAT library not found";
}

void ethercat_bridge::cleanup() {
    std::cout << "EtherCAT Bridge stub cleanup" << std::endl;
}

} // namespace ruiyan::rh6