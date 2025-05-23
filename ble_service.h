#ifndef _BLE_SERVICE_H_
#define _BLE_SERVICE_H_

#include <stdint.h>
#include <vector>

// 服务特征值
struct CharacteristicData {
    uint16_t        uuid;
    uint16_t        handle;
    uint8_t         property;
};

// 设备服务,一个服务可能包含多个特征
struct ServiceData {
    uint16_t        uuid;
    uint16_t        startHandle;
    uint16_t        endHandle;
    std::vector<CharacteristicData> chrData;
};


#endif