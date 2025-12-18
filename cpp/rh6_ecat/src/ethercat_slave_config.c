#include "rh6_ecat/ethercat_slave_config.h"
#include <string.h>

// 从站1-6的PDO配置（手部关节）
ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 7, slave_1_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_1_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站2-6使用相同的配置
ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 7, slave_2_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_2_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_2_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_2_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站3-6配置（复制相同的结构）
ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 7, slave_3_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_3_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_3_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站4-6配置
ec_pdo_entry_info_t slave_4_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_4_pdos[] = {
    {0x1600, 7, slave_4_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_4_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_4_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_4_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_4_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站5-6配置
ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_5_pdos[] = {
    {0x1600, 7, slave_5_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_5_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_5_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_5_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站6配置
ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Controlword */
    {0x607a, 0x00, 32}, /* Target position */
    {0x60ff, 0x00, 32}, /* Target velocity */
    {0x6071, 0x00, 16}, /* Target torque */
    {0x6060, 0x00, 16}, /* Modes of operation */
    {0x2007, 0x0c, 16}, /* pvtkp */
    {0x2007, 0x0e, 16}, /* pvtkd */
    {0x6041, 0x00, 16}, /* Statusword */
    {0x6064, 0x00, 32}, /* Position actual value */
    {0x606c, 0x00, 32}, /* Velocity actual value */
    {0x6077, 0x00, 16}, /* Torque actual value */
    {0x6061, 0x00, 16}, /* Modes of operation display */
    {0x200a, 0x05, 32}, /* Err Code */
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1600, 7, slave_6_pdo_entries + 0}, /* RxPdoMappingCsx */
    {0x1a00, 6, slave_6_pdo_entries + 7}, /* TxPdoMappingCsx */
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_6_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_6_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

// 从站配置表
slave_config_t slave_configs[MAX_SLAVES] = {
    // 从站1-6：手部关节（XNDDrive）
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_1_syncs},
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_2_syncs},
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_3_syncs},
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_4_syncs},
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_5_syncs},
    {XNDDrive_VENDOR_ID, XNDDrive_PRODUCT_CODE, XNDDrive_REVISION, slave_6_syncs},
    // 其他从站配置...
};

// 初始化从站配置
int init_slave_configs(void) {
    // 这里可以添加初始化逻辑
    return 0;
}

// 获取从站配置
int get_slave_config(int slave_index, slave_config_t *config) {
    if (slave_index < 0 || slave_index >= MAX_SLAVES || !config) {
        return -1;
    }
    
    *config = slave_configs[slave_index];
    return 0;
}

// 设置手部模式
int set_hand_mode(int hand_index, hand_mode_t mode) {
    if (hand_index < 0 || hand_index >= 2) {
        return -1; // 只支持左手(0)和右手(1)
    }
    
    if (mode < HAND_MODE_0 || mode > HAND_MODE_2) {
        return -1; // 无效模式
    }
    
    // 根据模式设置相应的控制参数
    switch (mode) {
        case HAND_MODE_0: // 位置控制模式
            // 设置位置控制参数
            // 实际实现需要通过EtherCAT PDO设置从站的modes_of_operation
            break;
        case HAND_MODE_1: // 速度控制模式
            // 设置速度控制参数
            break;
        case HAND_MODE_2: // 力矩控制模式
            // 设置力矩控制参数
            break;
        default:
            return -1;
    }
    
    return 0;
}
