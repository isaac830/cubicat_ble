#ifndef _BLE_SERVICE_H_
#define _BLE_SERVICE_H_

#include <stdint.h>
#include <vector>
#include <functional>
#include "ble_protocol.h"

// 服务特征值
// ops: 一个特征值可能包含多个操作码
struct CharacteristicData {
    uint16_t        uuid;
    uint16_t        handle;
    uint8_t         property;
    uint16_t        cccdHandle;
    std::vector<uint32_t> opCodes;
};

// 设备服务,一个服务可能包含多个特征
struct ServiceData {
    uint16_t        uuid;
    uint16_t        startHandle;
    uint16_t        endHandle;
    std::vector<CharacteristicData> chrData;
};

#endif