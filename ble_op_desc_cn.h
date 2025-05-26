#ifndef _BLE_OP_DESC_CN_H_
#define _BLE_OP_DESC_CN_H_
#include <map>
#include <string>
#include "ble_service_defines.h"


static std::map<int, std::string> OpDescCN = {
    {OP_LIGHT_SWITCH,       "用来控制灯的开关"}, 
    {OP_LIGHT_COLOR,        "用来调节灯的颜色"}, 
    {OP_LIGHT_BRIGHTNESS,   "用来调节灯的亮度"}
};


#endif