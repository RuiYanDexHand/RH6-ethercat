#pragma once

#include <cstdint>
#include <cstring>

namespace ruiyan::rh6 {

// 睿研手部控制相关类型定义
// 基于 hand_ecat/ryhand_ecat/ryhand6_b3.h 和 RyHandLib.h

// 伺服状态信息
struct RyServoInfo {
    uint64_t :8;         // 前面的CMD
    uint64_t status:8;   // 故障状态
    uint64_t P:12;       // 当前位置，0-4095 对应 0到满行程
    uint64_t V:12;       // 当前速度，-2048~2047 单位 0.001行程/s
    uint64_t I:12;       // 当前电流，-2048~2047 单位 0.001A
    uint64_t F:12;       // 当前位置，0-4095 对应手指压力传感器Adc原始值
};

// 伺服命令 - 使用pack(1)确保字节对齐正确
#pragma pack(push, 1)
struct RyServoCmd {
    uint8_t cmd;         // 数据内容  
    uint16_t usPos;      // 目标位置 (小端序)
    uint16_t usSpd;      // 目标过程速度 (小端序)
    uint16_t usMaxCur;   // 目标最大电流 (小端序)
    uint8_t res;         // 保留
};
#pragma pack(pop)

// 手部消息结构
struct RyHandMsg {
    uint32_t ulId;       // id
    uint8_t ucLen;       // 数据长度
    uint8_t pucDat[64];  // 数据内容
};

// 手部数据
struct RyHandData {
    uint16_t len;     
    uint8_t buff[24]; 
};

// 手部接收PDO
struct RyHandRxPdo {
    uint16_t RxLen_1;     
    uint8_t RxData_1[24];        
    uint16_t RxLen_2;     
    uint8_t RxData_2[24];    
    uint16_t RxLen_3; 
    uint8_t RxData_3[24];  
    uint16_t RxLen_4; 
    uint8_t RxData_4[24];  
    uint16_t RxLen_5; 
    uint8_t RxData_5[24];  
    uint16_t RxLen_6; 
    uint8_t RxData_6[24];  
};

// 手部发送PDO
struct RyHandTxPdo {
    uint16_t TxLen_1; 
    uint8_t TxData_1[24];  
    uint16_t TxLen_2; 
    uint8_t TxData_2[24]; 
    uint16_t TxLen_3; 
    uint8_t TxData_3[24]; 
    uint16_t TxLen_4; 
    uint8_t TxData_4[24];  
    uint16_t TxLen_5; 
    uint8_t TxData_5[24]; 
    uint16_t TxLen_6; 
    uint8_t TxData_6[24]; 
};

// 睿研CAN伺服总线结构
struct RyCanServoBus {
    volatile uint16_t* pusTicksMs; // ms计数器地址
    uint16_t usTicksPeriod;        // ms记数器的周期
    uint16_t usHookNum;
    uint16_t usListenNum;
    void* pstuHook;        // hook 列表首地址
    void* pstuListen;      // listen 列表首地址
    int8_t (*pfunWrite)(RyHandMsg stuMsg);  // CAN设备写接口地址
};

// 伺服数据 - 确保union正确对齐
#pragma pack(push, 1)
struct ServoData {
    union {
        RyServoCmd stuCmd;
        RyServoInfo stuInfo;
        uint8_t pucDat[64];
    };
};
#pragma pack(pop)

// 消息钩子
struct MsgHook {
    volatile uint8_t ucEn;        // 使能开关
    volatile uint8_t ucAlive;     // hook生命值
    RyHandMsg* pstuMsg;
    void (*funCbk)(RyHandMsg stuMsg, void* para);
};

// 消息监听
struct MsgListen {
    MsgHook stuListen;
    ServoData stuRet;
    uint8_t ucConfidence;    // 数据可信度
};

} // namespace ruiyan::rh6

