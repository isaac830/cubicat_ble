/*
* @author       Isaac
* @date         2025-05-02
* @license      MIT License
* @copyright    Copyright (c) 2025 Deer Valley
* @description  BLE client for esp32 using nimBLE framework
*/
#ifndef _BLE_CLIENT_
#define _BLE_CLIENT_

#include <string>
#include <map>
#include <vector>
#include <functional>
#include <deque>
#include "ble_service.h"
#include "esp_timer.h"
#include "ble_protocol.h"

// GATT standard properties
#define PROPERTY_BROADCAST      0x01
#define PROPERTY_READ           0x02
#define PROPERTY_WRITE_NO_RSP   0x04
#define PROPERTY_WRITE_RSP      0x08

#define PROPERTY_NOTIFY         0x10
#define PROPERTY_INDICATE       0x20

// GATT standard services
#define CHR_NAME                0x2A00
#define CHR_APPEARANCE          0x2A01
#define CHR_PERIPHERAL_PREF     0x2A04

struct BLEDevice {
    const bool      connectable;
    const uint8_t   macAddr[6];
    const uint8_t   addrType;
    uint16_t        connHandle;
    std::string     name;
    uint16_t        vendor;
    std::vector<ServiceData> services;
};

using ConnectedCallback = std::function<void(uint16_t connId)>;
using MacAddr = uint8_t[6];
using ReadFunc = std::function<void(const ExBuffer&)>;

enum ScanPolicy {
    SCAN_ONLY_CUBICAT,
    SCAN_ALL_DEVICES
};

class BLEClient {
public:
    static BLEClient* getInstance() {
        static BLEClient instance;
        return &instance;
    }
    ~BLEClient();
    void init();
    void deinit();
    // @param autoConnect: if true, automatically connect to the first device found
    // @param connectDelay: if autoConnect is true, wait this many seconds before connecting
    //                      note: this is not the discovery duration, it is the delay after 
    //                      the first device is found, then the connection will begin
    void scan(ScanPolicy policy, bool autoConnect = true, int connectDelay = 2);
    const std::vector<BLEDevice>& getAllDevices();
    BLEDevice* getDevice(MacAddr addr);
    BLEDevice* getDevice(uint16_t connHandle);
    CharacteristicData* getCharacteristicByUUID(uint16_t connHandle, uint16_t charUUID);
    CharacteristicData* getCharacteristicByHanle(uint16_t connHandle, uint16_t handle);
    
    bool read(uint16_t connHandle, uint16_t charUUID, ReadFunc onRead);
    bool write(uint16_t connHandle, uint16_t charUUID, const BLEProtocol& protocol);

    bool hasService(uint16_t connHandle, uint16_t servUUID);
    void setConnectedCallback(ConnectedCallback onDeviceConnected) { m_connectedCB = onDeviceConnected; }
    void connect(const BLEDevice& device);
    // Internal functions
    void onScanResult(MacAddr addr, uint8_t addrType, std::string name, bool hasCubicatService,
         uint16_t vendor, bool connectable);
    void onDeviceConnected(MacAddr addr, uint16_t connHandle);
    void onServiceFound(uint16_t connHandle, uint16_t srvcUUID, uint16_t startHandle, uint16_t endHandle);
    void onCharacteristicFound(uint16_t connHandle, uint16_t srvcUUID, uint16_t charUUID, uint16_t handle, uint8_t property);
    void discoverCharacteristics(uint16_t connHandle, bool processNext);
private:
    BLEClient();
    BLEClient(BLEClient const&) = delete;

    bool write(uint16_t connHandle, uint16_t charUUID, const uint8_t* data, uint16_t dataLen);
    static void autoConnect(void* arg);
    const BLEDevice* deviceFound(MacAddr addr, uint8_t addrType, std::string name, uint16_t vendor, bool connectable);
    void getHandleRangeByCharHandle(uint8_t gattc_if, uint16_t charHandle, uint16_t* startHandle, uint16_t* endHandle);
    std::vector<BLEDevice>  m_devices;
    ConnectedCallback       m_connectedCB = nullptr;
    // 需要发现的服务队列，BLE只能依次发现
    std::deque<ServiceData> m_discServQueue;
    bool                    m_bServInDiscovering = false;
    bool                    m_bAutoConnect = false;
    int                     m_connectDelay = 0;
    static esp_timer_handle_t      m_sAutoConnectTimer;
    ScanPolicy              m_eScanPolicy;
};

#endif