#ifndef _EXAMPLE_CLIENT_
#define _EXAMPLE_CLIENT_

#include "ble_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ble_service_defines.h"

extern "C" void app_main() 
{
    BLEClient* bleClient = BLEClient::getInstance();
    bleClient->init();
    bleClient->scan(ScanPolicy::SCAN_ONLY_CUBICAT);
    bleClient->setConnectedCallback([=](uint16_t connHandle) {
        auto chr = bleClient->getCharacteristicByUUID(connHandle, CUBICAT_PROTOCOL_CHAR_UUID);
        // 检查特征是否有指定的操作，在这里就是开灯操作
        for (auto op : chr->opCodes) {
            if (op == OP_LIGHT_SWITCH) {
                bleClient->write(connHandle, CUBICAT_PROTOCOL_CHAR_UUID, BLEProtocol(OP_LIGHT_SWITCH, 1));
                bleClient->read(connHandle, CUBICAT_PROTOCOL_CHAR_UUID, [=](ExBuffer value) {
                    
                });
            }
        }
    });
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // 