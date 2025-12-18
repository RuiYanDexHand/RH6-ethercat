#ifndef _ETHERCAT_SLAVE_CONFIG_H_
#define _ETHERCAT_SLAVE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 条件编译：只有在找到 EtherCAT 库时才包含 ecrt.h
#ifdef ECAT_LIB_FOUND
#include "ecrt.h"
#endif

// EtherCAT 从站类型定义
#define RYHand_VENDOR_ID     0x00000001
#define RYHand_PRODUCT_CODE  0x00010000
#define RYHand_REVISION      0x00000001

#define XNDDrive_VENDOR_ID   0x00000001
#define XNDDrive_PRODUCT_CODE 0x00010000
#define XNDDrive_REVISION    0x00000001

#define ESC_Switch_VENDOR_ID 0x00000000
#define ESC_Switch_PRODUCT_CODE 0x00000000
#define ESC_Switch_REVISION  0x00000000

// 从站数量定义
#define MAX_SLAVES 34
#define ECAT_MAX_JOINTS 6

// PDO 条目信息结构（避免与 IGH Etherlab 冲突）
typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
} rh6_ec_pdo_entry_info_t;

// PDO 信息结构（避免与 IGH Etherlab 冲突）
typedef struct {
    uint16_t index;
    int n_entries;
    rh6_ec_pdo_entry_info_t *entries;
} rh6_ec_pdo_info_t;

// 同步信息结构（避免与 IGH Etherlab 冲突）
typedef struct {
    uint8_t index;
    uint8_t dir;
    uint8_t n_pdos;
    rh6_ec_pdo_info_t *pdos;
    uint8_t watch_dog;
} rh6_ec_sync_info_t;

// 从站配置结构
typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    rh6_ec_sync_info_t *syncs;
} slave_config_t;

// 手部模式定义
typedef enum {
    HAND_MODE_0 = 0,  // 位置控制模式
    HAND_MODE_1 = 1,  // 速度控制模式  
    HAND_MODE_2 = 2   // 力矩控制模式
} hand_mode_t;

// 从站配置数组声明
extern ec_pdo_entry_info_t slave_1_pdo_entries[];
extern ec_pdo_info_t slave_1_pdos[];
extern ec_sync_info_t slave_1_syncs[];

extern ec_pdo_entry_info_t slave_2_pdo_entries[];
extern ec_pdo_info_t slave_2_pdos[];
extern ec_sync_info_t slave_2_syncs[];

extern ec_pdo_entry_info_t slave_3_pdo_entries[];
extern ec_pdo_info_t slave_3_pdos[];
extern ec_sync_info_t slave_3_syncs[];

extern ec_pdo_entry_info_t slave_4_pdo_entries[];
extern ec_pdo_info_t slave_4_pdos[];
extern ec_sync_info_t slave_4_syncs[];

extern ec_pdo_entry_info_t slave_5_pdo_entries[];
extern ec_pdo_info_t slave_5_pdos[];
extern ec_sync_info_t slave_5_syncs[];

extern ec_pdo_entry_info_t slave_6_pdo_entries[];
extern ec_pdo_info_t slave_6_pdos[];
extern ec_sync_info_t slave_6_syncs[];

// 从站配置表
extern slave_config_t slave_configs[MAX_SLAVES];

// 函数声明
int init_slave_configs(void);
int get_slave_config(int slave_index, slave_config_t *config);
int set_hand_mode(int hand_index, hand_mode_t mode);

// 注意：PDO 配置暂时简化，避免与 IGH Etherlab 类型冲突

#ifdef __cplusplus
}
#endif

#endif // _ETHERCAT_SLAVE_CONFIG_H_
