/*
* @author       Isaac
* @date         2025-05-02
* @license      MIT License
* @copyright    Copyright (c) 2025 Deer Valley
* @description  BLE server for esp32 using nimBLE framework
*/
#include "ble_server.h"

#ifdef CONFIG_BT_ENABLED

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
#include "ble_protocol.h"
#include "ble_service_defines.h"

#define TAG "BLE_SERVER"

TaskHandle_t            g_slaveTask = nullptr;
EventGroupHandle_t      g_startEvent = xEventGroupCreate();

#define BLE_SERVER_START_BIT    BIT0


static int on_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    BLECharacteristic* chr = (BLECharacteristic*)arg;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
        {
            ExBuffer output;
            chr->onRead(output);
            if (output.size())
                os_mbuf_append(ctxt->om, output.data(), output.size());
            break;
        }
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            ExBuffer input;
            input.resize(ctxt->om->om_len);
            os_mbuf_copydata(ctxt->om, 0, input.size(), input.data());
            chr->onWrite(conn_handle, input);
        }
            break;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

static int on_dsc_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_DSC:
        {
            BLECharacteristic* chr = (BLECharacteristic*)arg;
            // Write the descriptor with opCodes
            const auto& dscs = chr->getOpCodes();
            os_mbuf_append(ctxt->om, dscs.data(), dscs.size() * sizeof(uint32_t));
        }
            break;

        case BLE_GATT_ACCESS_OP_WRITE_DSC:
            // TODO write
            break;
    }
    return 0;
}
// void NotifyOpCode(uint16_t connHandle, uint32_t opCode) {
//     std::vector<uint8_t> opCodes;
//     for (auto& opPair : m_opCallbacks) {
//         if (opPair.first != OP_SYS_CONNECT) {
//             opCodes.push_back(opPair.first);
//         }
//     }
//     struct os_mbuf *txom = ble_hs_mbuf_from_flat(opCodes.data(), opCodes.size());
//     int rc = ble_gatts_notify_custom(connHandle, m_data.handle, txom);
//     if (rc != 0) {
//         printf("Failed to send Notification: %d\n", rc);
//     }
// }
BLECharacteristic::BLECharacteristic(uint16_t uuid)
{
    m_chrUUID = BLE_UUID16_INIT(uuid);
    m_characteristicDef = {
        .uuid = &m_chrUUID.u,
        .access_cb = on_chr_access,
        .arg = this,
        .descriptors = nullptr,
        .flags = 0,
        .val_handle = &this->m_data.handle,
        .cpfd = nullptr
    };
}

void BLECharacteristic::onRead(ExBuffer& value) {
    if (m_onRead) {
        m_onRead(value);
    }
}

void BLECharacteristic::onWrite(uint16_t connHandle, const ExBuffer& value) {
    BLEProtocol protocol;
    auto ops = protocol.parse(value.data(), value.size());
    for (auto& opPair : ops) {
        if (m_opCallbacks.find(opPair.first) != m_opCallbacks.end()) {
            m_opCallbacks[opPair.first](connHandle, opPair.second);
        }
    }
}

void BLECharacteristic::addRead(OutboundFunc onRead) {
    m_onRead = onRead;
    m_data.property |= BLE_GATT_CHR_F_READ;
}

BLECharacteristic* BLECharacteristic::addOperation(uint32_t opCode, OperationFunc onWrite) {
    m_opCallbacks[opCode] = onWrite;
    m_data.property |= BLE_GATT_CHR_F_WRITE;
    return this;
}

const ble_gatt_chr_def& BLECharacteristic::getDef() {
    m_characteristicDef.flags = m_data.property;
    m_descriptorDefs.clear();
    m_dscUUID = BLE_UUID16_INIT(CUBICAT_DESCRIPTOR_UUID);
    ble_gatt_dsc_def dscDef = {
        .uuid = &m_dscUUID.u,
        .att_flags = BLE_ATT_F_READ,
        .min_key_size = 0,
        .access_cb = on_dsc_access,
        .arg = this
    };
    m_descriptorDefs.emplace_back(dscDef);
    m_descriptorDefs.push_back({ 0 }); // end
    m_characteristicDef.descriptors = m_descriptorDefs.data();
    return m_characteristicDef;
}
const std::vector<uint32_t> BLECharacteristic::getOpCodes() {
    m_opCodes.clear();
    for (auto& opPair : m_opCallbacks) {
        m_opCodes.push_back(opPair.first);
    }
    return m_opCodes;
}

//=============================================================================
// BLEService

BLEService::BLEService(uint16_t _uuid) {
    m_data.uuid = _uuid;
    m_svcUUID = BLE_UUID16_INIT(_uuid);
}

BLEService::~BLEService() {
    for (auto& chr : m_Characteristics) {
        delete chr;
    }
    m_Characteristics.clear();
}

BLECharacteristic* BLEService::createCharacteristic(uint16_t uuid) {
    BLECharacteristic* chr = new BLECharacteristic(uuid);
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
    // Set server MTU to 247 bytes, so notifications can be larger
    printf("BLE_ATT_MTU_MAX = %d\n", BLE_ATT_MTU_MAX);
    // ble_att_set_preferred_mtu(23);
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

uint8_t calculate_adv_length(const struct ble_hs_adv_fields *fields) {
    uint8_t total_len = 0;

    // 1. Flags 字段（固定 3 字节：1长度 + 1类型 + 1值）
    if (fields->flags != 0) {
        total_len += 3;  // 0x02 0x01 <flags>
    }

    // 2. 16-bit UUIDs 字段
    if (fields->uuids16 != NULL && fields->num_uuids16 > 0) {
        // 长度 = 1字节长度 + 1字节类型 + N*2字节UUID
        total_len += 1 + 1 + fields->num_uuids16 * 2;
    }

    // 3. 设备名称字段
    if (fields->name != NULL && fields->name_len > 0) {
        // 长度 = 1字节长度 + 1字节类型 + name_len字节数据
        total_len += 1 + 1 + fields->name_len;
    }

    // 4. 厂商数据字段
    if (fields->mfg_data != NULL && fields->mfg_data_len > 0) {
        // 长度 = 1字节长度 + 1字节类型 + mfg_data_len字节数据
        total_len += 1 + 1 + fields->mfg_data_len;
    }
    printf("Advertisement len = %d\n", total_len);
    // 确保不超过 BLE 4.x 的 31 字节限制
    return (total_len <= 31) ? total_len : 31;
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
        serviceUUIDs.emplace_back(service->getUUID16());
        ble_gatt_svc_def serviceDef = {
            .type = BLE_GATT_SVC_TYPE_PRIMARY,
            .uuid = &service->getUUID16().u,
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
    uint8_t vendor[2];
    vendor[0] = (uint8_t)(m_vendorId >> 8);
    vendor[1] = (uint8_t)(m_vendorId & 0xFF);
    // Advertising data should be 31 bytes maximum
    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP, 
        .uuids16 = &serviceUUIDs.data()[0],      
        .num_uuids16 = (uint8_t)serviceUUIDs.size(),
        .name = (uint8_t*)m_deviceName.c_str(),
        .name_len = (uint8_t)m_deviceName.length(),
        .mfg_data = vendor,
        .mfg_data_len = sizeof(vendor),
    };
    bool valid = calculate_adv_length(&fields);
    if (!valid) {
        ESP_LOGE(TAG, "Advertising data too long");
        return;
    }
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

#endif // CONFIG_BT_ENABLED