#ifndef _BLE_SERVICE_DEFINES_H_
#define _BLE_SERVICE_DEFINES_H_

#define CUBICAT_SERVICE_UUID    0xFF91


// Op code = (op id << 16) | value size

#define OP_DEFINE(category, action, id, valueSize) \
    const uint32_t OP_##category##_##action = (uint16_t)id << 16 | (uint16_t)valueSize;

OP_DEFINE(LIGHT, SWITCH,        0, 1)
OP_DEFINE(LIGHT, COLOR,         2, 3)
OP_DEFINE(LIGHT, BRIGHTNESS,    2, 1)

// OP_DEFINE(AUDIO, PLAYPAUSE,     200, 1)
// OP_DEFINE(AUDIO, VOLUME,        201, 1)

// OP_DEFINE(AC, TEMPERATURE,      300, 1)

#endif