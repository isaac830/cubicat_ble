#ifndef _BLE_SERVICE_DEFINES_H_
#define _BLE_SERVICE_DEFINES_H_

#define CUBICAT_SERVICE_UUID            0xFF91

#define CUBICAT_PROTOCOL_CHAR_UUID      0xFD91

#define CUBICAT_DESCRIPTOR_UUID         0x0002

// Op code = (op id << 16) | value size
#define OP_DEFINE(category, action, id, valueSize) \
    const unsigned int OP_##category##_##action = (uint16_t)id << 16 | (uint16_t)valueSize;

OP_DEFINE(LIGHT, SWITCH,        100, 1)
OP_DEFINE(LIGHT, COLOR,         101, 3)
OP_DEFINE(LIGHT, BRIGHTNESS,    102, 1)
OP_DEFINE(LIGHT, BREATH,        103, 1)
OP_DEFINE(LIGHT, RAINBOW,       104, 1)

// OP_DEFINE(AUDIO, PLAYPAUSE,     200, 1)
// OP_DEFINE(AUDIO, VOLUME,        201, 1)

// OP_DEFINE(AC, TEMPERATURE,      300, 1)

#endif