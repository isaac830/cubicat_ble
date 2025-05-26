/*
* @author       Isaac
* @date         2025-05-02
* @license      MIT License
* @copyright    Copyright (c) 2025 Deer Valley
* @description  BLE client for esp32 using nimBLE framework
*/
#include "ble_client.h"
#include "sdkconfig.h"

#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#include <string.h>
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
#include "ble_service_defines.h"

TaskHandle_t            g_hostTask = nullptr;

#define BLECENT_SVC_ALERT_UUID              0x1811
#define TAG "BLE_CLIENT"

extern "C" void ble_store_config_init();

static int gap_event_callback(struct ble_gap_event *event, void *arg);
static int chr_disc_callback(uint16_t conn_handle, const struct ble_gatt_error *error,
                          const struct ble_gatt_chr *chr, void *arg);

static void blemesh_on_reset(int reason)
{
    ESP_LOGI(TAG, "Resetting state; reason=%d\n", reason);
}

static void blemesh_on_sync(void)
{

}

int ble_gatt_attr_callback(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr,
                             void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG,"Error: Failed to read attribute; rc=%d", error->status);
        return error->status;
    }
    if (attr && attr->om) {
        // Current chunk data length
        uint16_t chunkLen = OS_MBUF_PKTLEN(attr->om);
        // Cache current chunk
#ifdef BLE_DEBUG
        printf("Received %d bytes at offset %d\n", chunkLen, attr->offset);
#endif
        // Check if there is more data， equal to MTU - 3 mean current package reached max shatt size,there could be more
        if (chunkLen == ble_att_mtu(conn_handle) - 3) {
            // Calculate next offset：current offset + current chunk length
            uint16_t nextOffset = attr->offset + chunkLen;
            ble_gattc_read_long(conn_handle, attr->handle, nextOffset, ble_gatt_attr_callback, arg);
        } else {
            ReadFunc onRead = *(ReadFunc*)arg;
            if (onRead) {
                ExBuffer buffer(attr->om->om_data, attr->om->om_data + attr->om->om_len);
                onRead(buffer);
            }
        }
    }

    return 0;
}

int ble_gatt_desc_callback(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr,
                             void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG,"Error: Failed to read descriptor; rc=%d", error->status);
        return error->status;
    }
    if (attr) {
        CharacteristicData* chr = (CharacteristicData*)arg;
        int c = attr->om->om_len / sizeof(uint32_t);
        unsigned int* data = (unsigned int*)attr->om->om_data;
        for (int i = 0; i < c; i++) {
            printf("OpCode: 0x%08x\n", data[i]);
            chr->opCodes.push_back(data[i]);
        }

    }
    return 0;
}

void ble_host_task(void *param)
{
    ble_hs_cfg.reset_cb = blemesh_on_reset;
    ble_hs_cfg.sync_cb = blemesh_on_sync;
    // ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_run();
    if (g_hostTask) {
        vTaskDelete(g_hostTask);
        g_hostTask = nullptr;
    }
}

static int blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i;
    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return 0;
    }
    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return 0;
    }
    /* The device has to advertise support for the Alert Notification
     * service (0x1811).
     */
    for (i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == BLECENT_SVC_ALERT_UUID) {
            return 1;
        }
    }

    return 0;
}

static int gap_event_callback(struct ble_gap_event *event, void *arg) {
    int rc;
    if (event->type == BLE_GAP_EVENT_DISC) {
        if (event->disc.addr.type != BLE_ADDR_PUBLIC) {
            return 0;
        }
        ble_gap_disc_desc *disc = &event->disc;
        printf("发现设备 data: ");
        for (int i = 0; i < disc->length_data; i++) {
            printf("%02X ", disc->data[i]);
        }
        printf("\ndata len: %d \n", disc->length_data);
        if (!disc->length_data) {
            return 0;
        }
        struct ble_hs_adv_fields fields;
        rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
        if (rc != 0) {
            ESP_LOGW(TAG, "广播数据解析失败\n");
            return 0;
        }
        auto name = std::string((const char*)fields.name, fields.name_len);
        bool connectable = disc->event_type == BLE_HCI_ADV_TYPE_ADV_IND || disc->event_type == BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_HD;
        uint16_t vendor = 0;
        if (fields.mfg_data_len >= 2) {
            vendor = fields.mfg_data[0] << 8 | fields.mfg_data[1];
        }
        // 查找广播的uuid16里面是否包含cubicat服务
        bool hasCubicatService = false;
        for (int i = 0; i < fields.num_uuids16; i++) {
            if (fields.uuids16[i].value == CUBICAT_SERVICE_UUID) {
                hasCubicatService = true;
                break;
            }
        }
        printf("发现设备 name: %s, connectable: %d, vendor: %d, hasCubicatService: %d\n", name.c_str(), connectable, vendor, hasCubicatService);
        BLEClient::getInstance()->onScanResult(disc->addr.val, disc->addr.type, name, hasCubicatService, vendor, connectable);
    } else if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status == 0) {
            // 连接成功后发起 MTU 协商
            ble_gattc_exchange_mtu(event->connect.conn_handle, nullptr, nullptr);
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG, "Failed to find connection; rc=%d", rc);
                return 0;
            }
            BLEClient::getInstance()->onDeviceConnected(desc.peer_id_addr.val, event->connect.conn_handle);
            // 保存连接信息
            printf("连接成功\n");
        } else {
            printf("连接失败\n");
        }
        
    } else if (event->type == BLE_GAP_EVENT_MTU) {
        printf("Current MTU size: %d\n", event->mtu.value);
    }
    return 0;
}

void connectToDevice(const uint8_t* macAddr, uint8_t addrType) {
    // 连接之前必须停止扫描
    int rc = ble_gap_disc_cancel();
    if (rc != 0) {
        MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }
    static const struct ble_gap_conn_params conn_params = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,
        .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = 0x0100,
        .min_ce_len = 0x0010,
        .max_ce_len = 0x0300,
    };
    ble_addr_t addr;
    addr.type = addrType;
    memcpy(addr.val, macAddr, sizeof(MacAddr));
    rc = ble_gap_connect(
        BLE_OWN_ADDR_PUBLIC, // 本地地址类型
        &addr,                // 目标设备地址
        BLE_HS_FOREVER,      // 持续尝试连接
        &conn_params,        
        gap_event_callback, 
        NULL
    );
    if (rc != 0) {
        printf("连接失败: %d addr: %02X:%02X:%02X:%02X:%02X:%02X type: %d\n", rc,
             macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5], addrType);
    }
}
// 发现服务器回调
static int serv_disc_callback(uint16_t conn_handle, const ble_gatt_error *error, const ble_gatt_svc *service, void *arg) {
    if (service) {
        printf("发现服务 uuid: 0x%04X start: 0x%04X end: 0x%04X\n", service->uuid.u16.value, service->start_handle, service->end_handle);
        if (error->status != 0) {
            ESP_LOGE(TAG,"Error: Failed to discover services; rc=%d", error->status);
            return error->status;
        }
        BLEClient::getInstance()->onServiceFound(conn_handle, service->uuid.u16.value, service->start_handle, service->end_handle);
    }
    return 0;
}
// 发现服务特征
static int chr_disc_callback(uint16_t conn_handle, const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg) {
    uint16_t* servId = (uint16_t*)arg;
    if (error->status == 0) {
        BLEClient::getInstance()->onCharacteristicFound(conn_handle, *servId, chr->uuid.u16.value, chr->val_handle, chr->properties);
    } else if (error->status == BLE_HS_EDONE) {
        delete servId;
        BLEClient::getInstance()->discoverCharacteristics(conn_handle, true);
    } else {
        printf("发现特征回调失败 status: %d\n", error->status);
    }
    return 0;
}
// 发现特征描述回调
static int desc_disc_callback(uint16_t conn_handle, const struct ble_gatt_error *error, uint16_t chr_val_handle,
                            const struct ble_gatt_dsc *dsc, void *arg)
{
    if (!dsc) {
        return 0;
    }
    printf("发现描述 uuid: 0x%04X char val handle: 0x%04X\n", dsc->uuid.u16.value, chr_val_handle);
    if (error->status != 0) {
        // status 10: BLE_HS_EBADDATA
        printf("发现描述回调失败 chr val handle: 0x%04X status: %d\n", chr_val_handle, error->status);
        return error->status;
    }
    auto chr = (CharacteristicData*)arg;
    if (dsc->uuid.u16.value == 0x2902) {
        printf("Found CCCD handle: 0x%04X\n", dsc->handle);
        chr->cccdHandle = dsc->handle;
        // 开启通知
        uint16_t enableFlag = 0x0000;
        if (chr->property & PROPERTY_NOTIFY) {
            enableFlag = 0x0001;
        } else if (chr->property & PROPERTY_INDICATE) {
            enableFlag = 0x0002;
        }
        if (enableFlag) {
            int rc = ble_gattc_write_flat(conn_handle, dsc->handle, &enableFlag, sizeof(enableFlag), nullptr, nullptr);
            if (rc != 0) {
                ESP_LOGE(TAG,"Error: Failed to write CCCD; rc=%d", rc);
            }
        }
    } else if (dsc->uuid.u16.value == CUBICAT_DESCRIPTOR_UUID) {
        ble_gattc_read(conn_handle, dsc->handle, ble_gatt_desc_callback, chr);
    }
    return 0;
}

esp_timer_handle_t BLEClient::m_sAutoConnectTimer = nullptr;

BLEClient::BLEClient() {
}

BLEClient::~BLEClient() {
    deinit();
}

void BLEClient::init() {
    auto ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to init nimble %d ", ret);
        return;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // bt_mesh_proxy_svcs_register();
    ble_store_config_init();

    xTaskCreatePinnedToCoreWithCaps(ble_host_task, "nimble_host", NIMBLE_HS_STACK_SIZE,
                            NULL, (configMAX_PRIORITIES - 4), &g_hostTask, NIMBLE_CORE, MALLOC_CAP_SPIRAM);
}

void BLEClient::deinit() {
#if CONFIG_BT_CONTROLLER_ENABLED
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
#endif
    if (m_sAutoConnectTimer) {
        esp_timer_stop(m_sAutoConnectTimer);
        esp_timer_delete(m_sAutoConnectTimer);
        m_sAutoConnectTimer = nullptr;
    }
}

void BLEClient::scan(ScanPolicy policy, bool autoConnect, int connectDelay) {
    m_eScanPolicy = policy;
    m_bAutoConnect = autoConnect;
    m_connectDelay = connectDelay;
    uint8_t own_addr_type;
    ble_hs_id_infer_auto(0, &own_addr_type);
    struct ble_gap_disc_params scan_params = {
        .itvl = 0x60,   // 扫描间隔 (单位: 0.625ms)
        .window = 0x30, // 扫描窗口 (单位: 0.625ms)
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,   // 主动扫描（获取广播数据）
        .filter_duplicates = 1, // 过滤重复设备
    };
    ESP_LOGI(TAG, "Starting scan...\n");
    ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &scan_params, gap_event_callback, NULL);
}

// Connect to all discovered devices
void BLEClient::autoConnect(void* arg) {
    const auto& devices = BLEClient::getInstance()->getAllDevices();
    for (const auto& device : devices) {
        if (device.connectable) {
            connectToDevice(device.macAddr, device.addrType);
        }
    }
    if (m_sAutoConnectTimer) {
        esp_timer_stop(m_sAutoConnectTimer);
        esp_timer_delete(m_sAutoConnectTimer);
        m_sAutoConnectTimer = nullptr;
    }
}

void BLEClient::connect(const BLEDevice& device) {
    if (device.connectable && device.connHandle == 0) {
        connectToDevice(device.macAddr, device.addrType);
    }
}

const BLEDevice* BLEClient::deviceFound(MacAddr addr, uint8_t addrType, std::string name, uint16_t vendor, bool connectable) {
    auto dev = getDevice(addr);
    if (dev) {
        if (dev->name.empty())
            dev->name = name;
        return dev;
    }
    BLEDevice device = {
        .connectable = connectable,
        .macAddr = {
            addr[0],
            addr[1],
            addr[2],
            addr[3],
            addr[4],
            addr[5],
        },
        .addrType = addrType,
        .connHandle = 0,
        .name = name,
        .vendor = vendor
    };
    // Start to connect after first discovery if set as auto connect
    if (m_devices.empty() && m_bAutoConnect) {
        const esp_timer_create_args_t timer_args = {
            .callback = &autoConnect,
            .arg = nullptr,
            .name = "AutoConnect",
        };
        esp_timer_create(&timer_args, &m_sAutoConnectTimer);
        esp_timer_start_once(m_sAutoConnectTimer, m_connectDelay * 1000 * 1000); //（Units：us）
    }
    return &m_devices.emplace_back(device);
}

const std::vector<BLEDevice>& BLEClient::getAllDevices() {
    return m_devices;
}  

BLEDevice* BLEClient::getDevice(MacAddr addr) {
    for (auto& device : m_devices) {
        if (memcmp(addr, device.macAddr, sizeof(MacAddr)) == 0) {
            return &device;
        }
    }
    return nullptr;
}
BLEDevice* BLEClient::getDevice(uint16_t connHandle) {
    for (auto& device : m_devices) {
        if (device.connHandle == connHandle) {
            return &device;
        }
    }
    return nullptr;
}

CharacteristicData* BLEClient::getCharacteristicByUUID(uint16_t connHandle, uint16_t uuid) {
    auto device = getDevice(connHandle);
    if (device) {
        for (auto& serv : device->services) {
            for (auto& characteristic : serv.chrData) {
                if (characteristic.uuid == uuid) {
                    return &characteristic;
                }
            }
        }
    }
    return nullptr;
}

CharacteristicData* BLEClient::getCharacteristicByHanle(uint16_t connHandle, uint16_t handle) {
    auto device = getDevice(connHandle);
    if (device) {
        for (auto& characteristic : device->services) {
            for (auto& characteristic : characteristic.chrData) {
                if (characteristic.handle == handle) {
                    return &characteristic;
                }
            }
        }
    }
    return nullptr;
}

void BLEClient::onDeviceConnected(MacAddr addr, uint16_t connHandle) {
    auto device = getDevice(addr);
    if (device) {
        device->connHandle = connHandle;
        int rc = ble_gattc_disc_all_svcs(device->connHandle, serv_disc_callback, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG,"Error: Failed to discover services; rc=%d", rc);
        }
    }
}
void BLEClient::onServiceFound(uint16_t conn_handle, uint16_t uuid, uint16_t start_handle, uint16_t end_handle) {
    auto device = getDevice(conn_handle);
    if (device) {
        device->services.emplace_back(ServiceData(uuid, start_handle, end_handle));
        m_discServQueue.emplace_back(ServiceData(uuid, start_handle, end_handle));
        discoverCharacteristics(conn_handle, false);
    }
}

void BLEClient::onScanResult(MacAddr addr, uint8_t addrType, std::string name, bool hasCubicatService,
    uint16_t vendor, bool connectable) 
{
    // 过滤掉非cubicat设备
    if (m_eScanPolicy == ScanPolicy::SCAN_ONLY_CUBICAT && !hasCubicatService) {
        return;
    }
    BLEClient::getInstance()->deviceFound(addr, addrType, name, vendor, connectable);
}

void BLEClient::discoverCharacteristics(uint16_t connHandle, bool processNext) {
    if (processNext) {
        if (m_discServQueue.empty()) {
            m_bServInDiscovering = false;
            // 所有特征发现完成
            if (m_connectedCB) {
                m_connectedCB(connHandle);
            }
            return;
        }
    } else {
        if (m_bServInDiscovering) {
            return;
        }
    }
    auto curServ = m_discServQueue.front();
    m_discServQueue.pop_front();
    uint16_t* srvcUUID = new uint16_t;
    *srvcUUID = curServ.uuid;
    int rc = ble_gattc_disc_all_chrs(connHandle, curServ.startHandle, curServ.endHandle, chr_disc_callback, srvcUUID);
    if (rc != 0) {
        ESP_LOGE(TAG,"Error: Failed to discover characteristics; rc=%d", rc);
        return;
    }
    m_bServInDiscovering = true;
}
void BLEClient::onCharacteristicFound(uint16_t connHandle, uint16_t servUUID, uint16_t charUUID, uint16_t handle, uint8_t property) {
    auto device = getDevice(connHandle);
    if (device) {
        printf("发现特征: UUID=0x%04X, property=0x%02X, handle=%d\n", charUUID, property, handle);
        for (auto& serv : device->services) {
            if (serv.uuid == servUUID) {
                auto& chr = serv.chrData.emplace_back(CharacteristicData(charUUID, handle, property, 0, std::vector<uint32_t>()));
                if (charUUID == CUBICAT_PROTOCOL_CHAR_UUID) {
                    // Discover descriptors
                    int rc = ble_gattc_disc_all_dscs(connHandle, serv.startHandle, serv.endHandle, desc_disc_callback, &chr);
                    if (rc != 0) {
                        ESP_LOGE(TAG,"Error: Failed to discover descriptors; rc=%d", rc);
                    }
                }
                return;
            }
        }
    }
}
void
print_bytes(const uint8_t *bytes, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        printf("%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void
print_mbuf(const struct os_mbuf *om)
{
    printf("MBUF: ");
    int colon;
    colon = 0;
    while (om != NULL) {
        if (colon) {
            printf(":");
        } else {
            colon = 1;
        }
        print_bytes(om->om_data, om->om_len);
        om = SLIST_NEXT(om, om_next);
    }
    printf("\n");
}

bool BLEClient::read(uint16_t connHandle, uint16_t charUUID, ReadFunc onRead) {
    auto chr = getCharacteristicByUUID(connHandle, charUUID);
    if (chr) {
        // 协商MTU读取256字节，暂时不需要ble_gattc_read_long分片读取,如果有需要再修改
        // ble_gattc_read_long(connHandle, chr->handle, 0, ble_gatt_attr_callback, &onRead);
        ble_gattc_read(connHandle, chr->handle, ble_gatt_attr_callback, &onRead);
        return true;
    }
    return false;
}

bool BLEClient::write(uint16_t connHandle, uint16_t charUUID, const uint8_t* data, uint16_t dataLen) {
    auto chr = getCharacteristicByUUID(connHandle, charUUID);
    if (chr) {
        bool hasWriteRsp = chr->property & PROPERTY_WRITE_RSP;
        bool hasWriteNoRsp = chr->property & PROPERTY_WRITE_NO_RSP;
        if (hasWriteRsp || hasWriteNoRsp) {
            auto om = ble_hs_mbuf_from_flat(data, dataLen);
            if (!om) {
                ESP_LOGE(TAG,"om buffer alloc failed");
                return false;
            }
#ifdef BLE_DEBUG
            printf("write characteristic: %04X, handle: %d, length: %d\n", chr->uuid, chr->handle, dataLen);
            print_mbuf(om);
#endif  
            printf("has response: %d, has no response: %d\n", hasWriteRsp, hasWriteNoRsp);
            ble_gattc_write(connHandle, chr->handle, om, hasWriteRsp?ble_gatt_attr_callback:NULL, nullptr);
            return true;
        }
    }
    return false;
}

bool BLEClient::write(uint16_t connHandle, uint16_t charUUID, const BLEProtocol& protocol) {
    return write(connHandle, charUUID, protocol.getBuffer().data(), protocol.getBuffer().size());
}

bool BLEClient::hasService(uint16_t connHandle, uint16_t servUUID) {
    auto device = getDevice(connHandle);
    if (device) {
        for (auto& serv : device->services) {
            if (serv.uuid == servUUID) {
                return true;
            }
        }
    }
    return false;
}

#endif // 