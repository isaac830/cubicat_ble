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

// @param connHandle: connection handle
// @param opData: operation data
using OperationFunc = std::function<void(uint16_t connHandle, const OPData& opData)>;
using OutboundFunc = std::function<void(ExBuffer&)>;

class BLECharacteristic {
    friend class BLEService;
public:
    BLECharacteristic() = delete;
    
    BLECharacteristic* addOperation(uint32_t opCode, OperationFunc onWrite);
    // void addNotify(NotifyChrFunc onNotify); // TODO
    void addRead(OutboundFunc onRead);
    bool hasProperty(uint8_t property) { return m_data.property & property; }
    // Internal use only
    const ble_gatt_chr_def& getDef();
    const std::vector<uint32_t> getOpCodes();
    void onRead(ExBuffer& value);
    void onWrite(uint16_t connHandle, const ExBuffer& input);
private:
    BLECharacteristic(uint16_t uuid);
    CharacteristicData  m_data = {0};
    ble_uuid16_t        m_chrUUID;
    ble_uuid16_t        m_dscUUID;
    OutboundFunc        m_onRead = nullptr;
    ble_gatt_chr_def    m_characteristicDef;
    std::vector<ble_gatt_dsc_def>    m_descriptorDefs;
    std::unordered_map<uint32_t, OperationFunc>  m_opCallbacks;
    std::vector<uint32_t>                       m_opCodes;
};

class BLEService {
    friend class BLEServer;
public:
    BLECharacteristic* createCharacteristic(uint16_t uuid);
    BLECharacteristic* getCharacteristic(uint16_t uuid);
    const std::vector<ble_gatt_chr_def>& getCharacteristicsDefs();
    const ble_uuid16_t& getUUID16() { return m_svcUUID; }
private:
    BLEService(uint16_t uuid);
    BLEService(BLEService& service) = delete;
    ~BLEService();

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

    void init(uint16_t vendorId, std::string deviceName = "Cubicat");
    void deinit();

    BLEService* createService(uint16_t uuid);
    
    void start();
private:
    std::vector<BLEService*>    m_services;
    std::string                 m_deviceName;
    uint16_t                    m_vendorId;
    std::vector<ble_gatt_svc_def> m_ServiceDefs;
};

#endif