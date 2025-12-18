#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <stdint.h>
#include <pthread.h>
#include <sys/types.h>

#define MAX_JOINTS 37
#define SHM_NAME "/ethercat_data"
#define SHM_SIZE sizeof(SharedData_t)

/**
 * @brief 共享内存数据结构
 * 
 * 该结构体定义了EtherCAT主从站通信的共享内存数据格式，
 * 包含同步控制、状态信息、关节控制数据和RyHand设备数据。
 * 
 * @struct SharedData_t
 */
typedef struct {
    pthread_mutex_t mutex;              ///< 互斥锁，用于保护共享数据的线程安全访问
    pthread_cond_t data_ready;          ///< 条件变量，用于数据就绪通知和线程同步
    
    volatile int data_valid;            ///< 数据有效性标志，指示共享数据是否有效
    volatile int shutdown_flag;         ///< 关闭标志，用于通知各线程退出
    
    uint64_t timestamp_ns;              ///< 时间戳（纳秒），记录数据更新时间
    
    /**
     * @brief EtherCAT主站状态信息
     */
    struct {
        uint32_t slaves_responding;    ///< 响应的从站数量
        uint8_t al_states;             ///< 应用层状态
        int link_up;                   ///< 链路状态（1表示链路正常）
        uint32_t working_counter;      ///< 工作计数器，用于检测通信完整性
        uint32_t expected_wc;          ///< 期望的工作计数器值,通常是一个从站读 +1，写 +2， 读写 +3
    } master_status;

    volatile int tx_data_cnt;           ///< 发送数据更新标志，
    volatile int rx_data_cnt;           ///< 接收数据更新标志，

    volatile int tx_nj;                 ///< 有效发送命令的关节数量
    volatile int rx_nj;                 ///< 有效接收数据的关节数量

    volatile int tx_nh;                 ///< 有效发送命令的手数量
    volatile int rx_nh;                 ///< 有效接收数据的手数量
    
    /**
     * @brief 关节控制和反馈数据数组
     */
    struct {
        /**
         * @brief 发送到从站的控制数据
         */
        struct {
            uint16_t controlword;           ///< 控制字，用于设备状态控制
            int32_t target_position;        ///< 目标位置（编码器单位）
            int32_t target_velocity;        ///< 目标速度（0.01rpm）
            int16_t target_torque;          ///< 目标扭矩（0.01Nm）
            int16_t modes_of_operation;     ///< 操作模式（位置 8 /速度 9 /扭矩控制 10 / PVT 15）
            int16_t pvtkp;                  ///< PVT模式比例增益参数
            int16_t pvtkd;                  ///< PVT模式微分增益参数
        } tx_data;

        /**
         * @brief 从从站接收的反馈数据
         */
        struct {
            uint16_t statusword;            ///< 状态字，指示设备当前状态
            int32_t position_actual_value;  ///< 实际位置反馈（编码器单位）
            int32_t velocity_actual_value;  ///< 实际速度反馈（0.01rpm）
            int16_t torque_actual_value;    ///< 实际扭矩反馈（0.01Nm）
            int16_t modes_of_operation_display; ///< 当前操作模式显示 （位置 8 /速度 9 /扭矩控制 10 / PVT 15）
            int32_t err_code;               ///< 错误代码，用于故障诊断
            uint32_t max_encode;            ///< 最大编码器值，对应一圈（2*pi）有多少个编码器值
        } rx_data;

    } joints[MAX_JOINTS];                   ///< 关节数据数组，支持最多MAX_JOINTS个关节
    
    /**
     * @brief RyHand灵巧手通信数据
     */
    struct {
        uint16_t tx_len[6];             ///< 发送数据长度数组，对应6个手指
        uint8_t tx_data[6][24];         ///< 发送数据缓冲区，每个手指最多24字节
        uint16_t rx_len[6];             ///< 接收数据长度数组，对应6个手指
        uint8_t rx_data[6][24];         ///< 接收数据缓冲区，每个手指最多24字节
    } ryhand[2];                        ///< RyHand数据数组，支持双手操作（左手和右手）
    
} SharedData_t;

#endif // SHARED_DATA_H

