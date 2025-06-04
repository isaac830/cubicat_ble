/*
* @author       Isaac
* @date         2025-05-02
* @license      MIT License
* @copyright    Copyright (c) 2025 Deer Valley
* @description  CUBICAT BLE framework transport protocol
*/
#ifndef _BLE_PROTOCOL_H_
#define _BLE_PROTOCOL_H_

#include <vector>
#include <stdint.h>
#include <string.h>
#include <map>

// Cubicat ble operation protocol
// Number Ops| Op code | Op value | Op code | Op value| ...

using ExBuffer = std::vector<uint8_t>;

struct OPData
{
    uint32_t    opCode;
    ExBuffer    value;
};


class BLEProtocol
{
public:
    BLEProtocol() = default;
    template<typename T>
    BLEProtocol(uint32_t opCode, T value) {write(opCode, value);}
    const std::map<uint32_t, OPData>& parse(const uint8_t* data, uint16_t len) {
        m_ops.clear();
        // Protocol has minimum 6 bytes: 1 byte for op count and 4 bytes for op Code and 1 byte minimum for value
        if (len < 6) { 
            return m_ops;
        }
        uint8_t opCount = data[0];
        uint8_t* ptr = (uint8_t*)data + 1;
        for (uint32_t i = 0; i < opCount; i++) {
            uint32_t op = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
            ptr += 4;
            uint16_t valueSize = op & 0xFFFF;
            OPData opData;
            opData.value.resize(valueSize);
            memcpy(opData.value.data(), ptr, valueSize);
            ptr += valueSize;
            m_ops.emplace(op, opData);
        }
        return m_ops;
    }
    template<typename T>
    void write(uint32_t opCode, T value) {
        uint16_t valueSize = opCode & 0xFFFF;
        OPData data;
        data.opCode = opCode;
        data.value.resize(valueSize);
        memcpy(data.value.data(), &value, valueSize);
        m_ops[opCode] = data;
    }
    std::vector<uint8_t> getBuffer() const {
        std::vector<uint8_t> buffer;
        buffer.push_back(0);
        for (auto& op : m_ops) {
            buffer.push_back((uint8_t)(op.first >> 24));
            buffer.push_back((uint8_t)(op.first >> 16));
            buffer.push_back((uint8_t)(op.first >> 8));
            buffer.push_back((uint8_t)op.first & 0xFF);
            buffer.insert(buffer.end(), op.second.value.begin(), op.second.value.end());
            buffer[0]++;
        }
        return buffer;
    }
private:
    std::map<uint32_t, OPData>  m_ops;
};


#endif