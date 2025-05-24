#include "ble_server.h"

// #ifdef CONFIG_BT_ENABLED

#include <string.h>
extern "C" {
#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "nimble/ble.h"
#include <nimble/nimble_port_freertos.h>
#include <nimble/nimble_port.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <host/ble_hs_id.h>
#include "esp_nimble_cfg.h"
#include "host/ble_hs.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "services/dis/ble_svc_dis.h"   
}
#include "ble_service_defines.h"

#define TAG "BLE_SERVER"

TaskHandle_t            g_slaveTask = nullptr;
EventGroupHandle_t      g_startEvent = xEventGroupCreate();

#define BLE_SERVER_START_BIT    BIT0


static int char_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BLECharacteristic* chr = (BLECharacteristic*)arg;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
        {
            ChrValue value;
            chr->getOnRead()(value);
            os_mbuf_append(ctxt->om, &value.val, value.len);
            break;
        }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            ChrValue value;
            value.len = ctxt->om->om_len;
            os_mbuf_copydata(ctxt->om, 0, value.len, &value.val);
            chr->getOnWrite()(value);
        }
            break;
        case BLE_GATT_ACCESS_OP_READ_DSC:
        {

        }
            break;
        case BLE_GATT_ACCESS_OP_WRITE_DSC:
        {

        }
            break;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

BLECharacteristic::BLECharacteristic(uint16_t uuid, ReadChrFunc onRead, WriteChrFunc onWrite)
: m_onRead(onRead), m_onWrite(onWrite)
{
    m_bleUUID = BLE_UUID16_INIT(uuid);
    m_characteristicDef = {
        .uuid = &m_bleUUID.u,
        .access_cb = char_access,
        .arg = this,
        .descriptors = nullptr,
        .flags = (ble_gatt_chr_flags)((onRead? BLE_GATT_CHR_F_READ:0x0) | (onWrite? BLE_GATT_CHR_F_WRITE:0x0)),
        .val_handle = &this->m_data.handle,
        .cpfd = nullptr
    };
    printf("BLECharacteristic uuid: %p  getDef: %p\n", m_characteristicDef.uuid, getDef().uuid);
}

//=============================================================================
// BLEService

BLEService::BLEService(uint16_t _uuid) {
    m_data.uuid = _uuid;
    m_bleUUID = BLE_UUID16_INIT(_uuid);
}

BLEService::~BLEService() {
    for (auto& chr : m_Characteristics) {
        delete chr;
    }
    m_Characteristics.clear();
}

BLECharacteristic* BLEService::createCharacteristic(uint16_t uuid, ReadChrFunc onRead, WriteChrFunc onWrite) {
    BLECharacteristic* chr = new BLECharacteristic(uuid, onRead, onWrite);
    m_Characteristics.emplace_back(chr);
    return chr;
}

BLECharacteristic* BLEService::getCharacteristic(uint16_t uuid) {
    for (auto& chr : m_Characteristics) {
        if (chr->m_data.uuid == uuid) {
            return chr;
        }
    }
    return nullptr;
}

const std::vector<ble_gatt_chr_def>& BLEService::getCharacteristicsDefs() {
    m_CharacteristicDefs.clear();
    for (auto& chr : m_Characteristics) {
        m_CharacteristicDefs.push_back(chr->getDef());      
    }
    m_CharacteristicDefs.push_back({ 0 });
    return m_CharacteristicDefs;
}

void ble_server_task(void *param)
{
    xEventGroupWaitBits(g_startEvent, BLE_SERVER_START_BIT, true, true, portMAX_DELAY);
    nimble_port_run();
    if (g_slaveTask) {
        vTaskDelete(g_slaveTask);
        g_slaveTask = nullptr;
    }
}

BLEServer::BLEServer()
{
}

BLEServer::~BLEServer()
{
    for (auto& s : m_services) {
        delete s;
    }
}

void BLEServer::init(uint16_t vendorId, std::string deviceName)
{
    m_deviceName = deviceName;
    m_vendorId = vendorId;
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // Initialize NimBLE
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"Failed to init nimble %d ", err);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();

    xTaskCreatePinnedToCore(ble_server_task, "nimble_server", NIMBLE_HS_STACK_SIZE,
                            NULL, (configMAX_PRIORITIES - 4), &g_slaveTask, NIMBLE_CORE);
}

void BLEServer::deinit()
{
}

BLEService* BLEServer::createService(uint16_t uuid)
{  
    auto service = new BLEService(uuid);
    m_services.emplace_back(service);
    return service;
}

void BLEServer::start()
{
    int rc ;
    if (m_services.empty()) {
        ESP_LOGE(TAG, "BLE server has NO services! quit start...");
        return;
    }
    std::vector<ble_uuid16_t> serviceUUIDs = {
        // Device Information Service
        BLE_UUID16_INIT(BLE_SVC_DIS_UUID16),
    };
    // Register standard DIS services
    ble_svc_dis_init();
    ble_svc_dis_manufacturer_name_set("Cubicat");
    if (!m_deviceName.empty()) {
        ble_svc_gap_device_name_set(m_deviceName.c_str());
    }
    // Register custom services
    m_ServiceDefs.clear();
    for (auto& service : m_services) {
        const auto& defs = service->getCharacteristicsDefs();
        serviceUUIDs.emplace_back(service->getBLEUUID());
        ble_gatt_svc_def serviceDef = {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &service->getBLEUUID().u,
            .characteristics = &defs.data()[0],
        };
        m_ServiceDefs.emplace_back(serviceDef);
    }
    m_ServiceDefs.push_back({ 0 });
    rc = ble_gatts_count_cfg(m_ServiceDefs.data());
    if (rc != 0) {
        ESP_LOGE(TAG,"Error: Failed to count configuration; rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(m_ServiceDefs.data());
    if (rc != 0) {
        ESP_LOGE(TAG,"Error: Failed to add services; rc=%d", rc);
        return;
    }
    // Adv parameters
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,  // 可连接的非定向广播
        .disc_mode = BLE_GAP_DISC_MODE_GEN,  // 通用发现模式
        .itvl_min = 0x0025,
        .itvl_max = 0x0050,
        .channel_map = 0,
        .filter_policy = 0,
    };
    // Notify BLE protocol stack that we are ready to start
    xEventGroupSetBits(g_startEvent, BLE_SERVER_START_BIT);
    // Delay to allow BLE stack to start
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // Advertising data
    uint8_t mfd[4];
    mfd[0] = (uint8_t)(CUBICAT_SERVICE_UUID >> 8);
    mfd[1] = (uint8_t)(CUBICAT_SERVICE_UUID & 0xFF);
    mfd[2] = (uint8_t)(m_vendorId >> 8);
    mfd[3] = (uint8_t)(m_vendorId & 0xFF);
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP, 
        .uuids16 = &serviceUUIDs.data()[0],      
        .num_uuids16 = (uint8_t)serviceUUIDs.size(),
        .mfg_data = mfd,
        .mfg_data_len = 4
    };
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields err: %d", rc);
        return;
    }
    rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,      // 使用公共地址
        nullptr,                  // 无特定的对端地址
        BLE_HS_FOREVER,           
        &adv_params,              
        nullptr,                     
        nullptr                    
    );
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started successfully");
    }
}

// #endif // CONFIG_BT_ENABLED