/*
* @author       Isaac
* @date         2025-05-02
* @license      MIT License
* @copyright    Copyright (c) 2025 Deer Valley
* @description  BLE server for esp32 using nimBLE framework
*/
#ifndef _BLE_SERVER_
#define _BLE_SERVER_

#include "ble_service.h"
#include "host/ble_gatt.h"
#include <vector>
#include <string>
#include <functional>
#include <unordered_map>
#include "ble_protocol.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <freertos/timers.h> 
#include <atomic>

// @param connHandle: connection handle
// @param opData: operation data
using OperationFunc = std::function<void(uint16_t connHandle, const OPData& opData)>;
using OutboundFunc = std::function<void(ExBuffer&)>;
using NotifyFunc = std::function<void(void)>;
using IndicateFunc = std::function<void(uint16_t connHandle)>;

class BLECharacteristic {
    friend class BLEService;
public:
    BLECharacteristic() = delete;
    
    BLECharacteristic* addOperation(uint32_t opCode, OperationFunc onWrite);
    void setNotify(const uint8_t* value, uint16_t valueLen, uint16_t intervalMS);
    void setIndicate(const uint8_t* value, uint16_t valueLen, uint16_t intervalMS);
    void addRead(OutboundFunc onRead);

    bool hasProperty(uint8_t property) { return m_data.property & property; }
    uint16_t getValueHandle() { return m_data.handle; }
    // Internal use only
    const std::vector<uint32_t> getOpCodes();
    void onRead(ExBuffer& value);
    void onWrite(uint16_t connHandle, const ExBuffer& input);
private:
    BLECharacteristic(uint16_t uuid);
    const ble_gatt_chr_def& getDef();
    void tick(uint16_t connHandle);
    void reset();
    CharacteristicData  m_data = {0};
    ble_uuid16_t        m_chrUUID;
    ble_uuid16_t        m_dscUUID;
    OutboundFunc        m_onRead = nullptr;
    uint16_t            m_indicateInterval = 0;
    uint16_t            m_notifyInterval = 0;
    std::vector<uint8_t> m_notifyValue;
    std::vector<uint8_t> m_indicateValue;
    ble_gatt_chr_def    m_characteristicDef;
    std::vector<ble_gatt_dsc_def>    m_descriptorDefs;
    std::unordered_map<uint32_t, OperationFunc>  m_opCallbacks;
    std::vector<uint32_t>                       m_opCodes;
    uint32_t            m_nextNotifyTime = 0;
    uint32_t            m_nextIndicateTime = 0;
};

class BLEService {
    friend class BLEServer;
public:
    BLECharacteristic* createCharacteristic(uint16_t uuid);
    BLECharacteristic* getCharacteristic(uint16_t uuid);

private:
    BLEService(uint16_t uuid);
    BLEService(BLEService& service) = delete;
    ~BLEService();
    const ble_uuid16_t& getUUID16() { return m_svcUUID; }
    const std::vector<ble_gatt_chr_def>& getCharacteristicsDefs();
    void tick(uint16_t connHandle);
    void reset();

    ServiceData         m_data;
    ble_uuid16_t        m_svcUUID;
    std::vector<BLECharacteristic*>     m_Characteristics;
    std::vector<ble_gatt_chr_def>       m_CharacteristicDefs;
};

class BLEServer
{
public:
    BLEServer();
    ~BLEServer();

    void init(uint16_t vendorId, std::string deviceName = "Cubicat", uint16_t clientTimeoutMS = 5000);
    BLEService* createService(uint16_t uuid);
    BLEService* getService(uint16_t uuid);
    void start();
    void shutdown();
    // Internal use only
    void onConnect(uint16_t connHandle);
    void onDisconnect();
    void resetHeartbeatTimer();
    // Returns true if the server is still running
    bool tick();
private:
    void startAdvertising();
    std::vector<BLEService*>    m_services;
    std::string                 m_deviceName;
    uint16_t                    m_vendorId;
    std::vector<ble_gatt_svc_def> m_serviceDefs;
    bool                        m_bConnected = false;
    uint32_t                    m_nlastHeartbeat = 0;
    uint32_t                    m_nClientConnHandle = 0;
    EventGroupHandle_t          m_startEvent;
    TimerHandle_t               m_timeoutTimer = nullptr;
    TaskHandle_t                m_mainTask = nullptr;
    TaskHandle_t                m_loopTask = nullptr;
    std::atomic<bool>           m_bShutdown = false;
    uint16_t                    m_timeoutMS;
};

#endif