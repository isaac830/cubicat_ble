#ifndef _BLE_SERVER_
#define _BLE_SERVER_

#include "ble_service.h"
#include "host/ble_gatt.h"
#include <vector>
#include <string>
#include <functional>

struct ChrValue {
    union {
        uint8_t* data;
        uint8_t  ui8;
        uint16_t ui16;
        int32_t  i32;
    } val;
    uint16_t len;
};

using ReadChrFunc = std::function<void(ChrValue&)>;
using WriteChrFunc = std::function<void(ChrValue)>;

// class ValueAccessor {
// public:
//     virtual ChrValue readValue() = 0;
//     virtual void writeValue(uint8_t* data, uint16_t dataLen) = 0;
// };

class BLECharacteristic {
    friend class BLEService;
public:
    BLECharacteristic() = delete;
    ReadChrFunc getOnRead() { return m_onRead; }
    WriteChrFunc getOnWrite() { return m_onWrite; }
    const ble_gatt_chr_def& getDef() { return m_characteristicDef;}
private:
    BLECharacteristic(uint16_t uuid, ReadChrFunc onRead, WriteChrFunc onWrite = nullptr);
    CharacteristicData  m_data;
    ble_uuid16_t        m_bleUUID;
    ReadChrFunc         m_onRead = nullptr;
    WriteChrFunc        m_onWrite = nullptr;
    ble_gatt_chr_def    m_characteristicDef;
};

class BLEService {
    friend class BLEServer;
public:
    BLECharacteristic* createCharacteristic(uint16_t uuid, ReadChrFunc onRead, WriteChrFunc onWrite);
    BLECharacteristic* getCharacteristic(uint16_t uuid);
    const std::vector<ble_gatt_chr_def>& getCharacteristicsDefs();
    const ble_uuid16_t& getBLEUUID() { return m_bleUUID; }
private:
    BLEService(uint16_t uuid);
    BLEService(BLEService& service) = delete;
    ~BLEService();

    ServiceData         m_data;
    ble_uuid16_t        m_bleUUID;
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