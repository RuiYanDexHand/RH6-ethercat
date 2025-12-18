#include "rh6_ecat/ethercat_slave_config.h"
#include <cstring>

// 简化的从站配置实现
slave_config_t slave_configs[MAX_SLAVES];

int init_slave_configs(void) {
    // 初始化默认配置
    for (int i = 0; i < MAX_SLAVES; i++) {
        slave_configs[i].vendor_id = 0x00000001;
        slave_configs[i].product_code = 0x00010000;
        slave_configs[i].revision = 0x00000001;
        slave_configs[i].syncs = nullptr;
    }
    return 0;
}

int get_slave_config(int slave_index, slave_config_t *config) {
    if (slave_index < 0 || slave_index >= MAX_SLAVES || !config) {
        return -1;
    }
    
    *config = slave_configs[slave_index];
    return 0;
}

int set_hand_mode(int hand_index, hand_mode_t mode) {
    // 简化的手部模式设置
    return 0;
}

// 注意：PDO 配置暂时简化，避免与 IGH Etherlab 类型冲突
