#ifndef _BLE_PROTOCOL_H_
#define _BLE_PROTOCOL_H_

#include <vector>
#include <stdint.h>
#include <string>
#include <map>

// Cubicat ble operation protocol
// Number Ops| Op code | Op value | Op code | Op value| ...

struct OPData
{
    uint32_t    opCode;
    union
    {
        uint8_t     ui8;
        uint16_t    ui16;
        uint32_t    ui32;
        uint8_t*    data;
    } value;
    uint16_t    valueSize;
};


class BLEProtocol
{
public:
    std::map<uint32_t, OPData> read(uint8_t* data, uint16_t len) {
        std::map<uint32_t, OPData> ops;
        uint32_t opCount = data[0];
        uint8_t* ptr = data++;
        for (uint32_t i = 0; i < opCount; i++) {
            uint32_t op = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
            ptr += 4;
            uint16_t valueSize = op & 0xFFFF;
            OPData opData;
            opData.opCode = op;
            opData.valueSize = valueSize;
            opData.value.data = ptr;
            ptr += valueSize;
            ops[op] = opData;
        }
        return ops;
    }
    template<typename T>
    void write(uint32_t opCode, T value) {
        if (m_buffer.empty()) {
            // place holder for op count
            m_buffer.push_back(0x0);
        }
        uint16_t valueSize = opCode & 0xFFFF;
        // insert op code
        m_buffer.push_back((uint8_t)(opCode >> 24));
        m_buffer.push_back((uint8_t)(opCode >> 16));
        m_buffer.push_back((uint8_t)(opCode >> 8));
        m_buffer.push_back((uint8_t)opCode & 0xFF);
        // insert value
        uint8_t* dest = m_buffer.data() + m_buffer.size();
        m_buffer.resize(m_buffer.size() + valueSize);
        memcpy(dest, &value, valueSize);
        m_buffer[0]++;
    }
    const std::vector<uint8_t>& getBuffer() const {
        return m_buffer;
    }
private:
    std::vector<uint8_t>    m_buffer;
};


#endif