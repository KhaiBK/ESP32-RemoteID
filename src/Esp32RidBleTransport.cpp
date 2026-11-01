#include "Esp32RidBleTransport.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_timer.h>
#include <esp32-hal-bt.h>
#include <string.h>

#if !defined(CONFIG_BT_BLUEDROID_ENABLED) || !CONFIG_BT_BLUEDROID_ENABLED
#error "OpenDroneID legacy broadcaster requires the installed ESP32 Bluedroid stack"
#endif

static_assert(ESP_BLE_ADV_DATA_LEN_MAX == 31, "Expected 31-byte legacy advertising capacity");

Esp32RidBleTransport* Esp32RidBleTransport::instance = nullptr;

int32_t Esp32RidBleTransport::begin()
{
    initStep = "exclusive_stack_ownership";
    // Referencing btStarted() retains Arduino's Bluetooth HAL and its strong
    // btInUse() hook. Otherwise initArduino() selects the weak false hook and
    // releases all controller memory before setup(), preventing native init.
    // This is a read-only status query; initialization remains checked below.
    if (instance != nullptr || btStarted() || esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE ||
        esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_UNINITIALIZED)
        return ESP_ERR_INVALID_STATE;
    events = xQueueCreateStatic(QUEUE_LENGTH, sizeof(RidBleTransportEvent), queueStorage, &queueControl);
    if (events == nullptr) return ESP_ERR_NO_MEM;

    // Startup initialization is synchronous, before the FC UART is opened.
    // All advertising changes below are asynchronous and callback-confirmed.
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    config.mode = ESP_BT_MODE_BLE;
    initStep = "controller_init";
    esp_err_t status = esp_bt_controller_init(&config);
    if (status == ESP_OK)
    {
        initStep = "controller_enable_ble";
        status = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    }
    if (status == ESP_OK)
    {
        initStep = "bluedroid_init";
        status = esp_bluedroid_init();
    }
    if (status == ESP_OK)
    {
        initStep = "bluedroid_enable";
        status = esp_bluedroid_enable();
    }
    if (status == ESP_OK)
    {
        instance = this;
        initStep = "gap_callback";
        status = esp_ble_gap_register_callback(gapCallback);
    }
    if (status == ESP_OK)
    {
        initStep = "public_address";
        const uint8_t* deviceAddress = esp_bt_dev_get_address();
        if (deviceAddress == nullptr) status = ESP_FAIL;
        else memcpy(address, deviceAddress, sizeof(address));
    }
    if (status != ESP_OK)
    {
        instance = nullptr;
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) esp_bluedroid_disable();
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) esp_bluedroid_deinit();
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) esp_bt_controller_disable();
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) esp_bt_controller_deinit();
        return status;
    }

    params.adv_int_min = 0x00a0; // 160 * 0.625 ms = 100 ms, valid for NONCONN_IND.
    params.adv_int_max = 0x00a0;
    // Original ESP32 legacy advertising uses uncoded LE 1M PHY. No extended
    // advertising / coded-PHY APIs or GATT transport are used here.
    params.adv_type = ADV_TYPE_NONCONN_IND;
    params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    params.channel_map = ADV_CHNL_ALL;
    params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    initStep = "ready";
    return ESP_OK;
}

int32_t Esp32RidBleTransport::configure(const uint8_t* data, size_t length)
{
    if (instance != this) return ESP_ERR_INVALID_STATE;
    if (data == nullptr || length != RidBleLegacyFrame::ADVERTISING_SIZE) return ESP_ERR_INVALID_ARG;
    // This raw API copies exactly the supplied AD structure; no name/Flags are
    // appended. Controller must be stopped and completion awaited by the owner.
    return esp_ble_gap_config_adv_data_raw(const_cast<uint8_t*>(data), length);
}

int32_t Esp32RidBleTransport::start()
{
    if (instance != this) return ESP_ERR_INVALID_STATE;
    return esp_ble_gap_start_advertising(&params);
}

int32_t Esp32RidBleTransport::stop()
{
    if (instance != this) return ESP_ERR_INVALID_STATE;
    return esp_ble_gap_stop_advertising();
}

void Esp32RidBleTransport::enqueue(RidBleEventType type, int32_t status)
{
    // Same clock as Arduino millis(); captures callback time before loop logging.
    const RidBleTransportEvent event{type, status,
        static_cast<uint32_t>(esp_timer_get_time() / 1000), true};
    if (xQueueSend(events, &event, 0) != pdTRUE) overflow.store(true);
}

bool Esp32RidBleTransport::poll(RidBleTransportEvent& event)
{
    if (overflow.exchange(false))
    {
        event = {RidBleEventType::QUEUE_OVERFLOW, -1};
        return true;
    }
    return events != nullptr && xQueueReceive(events, &event, 0) == pdTRUE;
}

void Esp32RidBleTransport::gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    if (instance == nullptr || param == nullptr) return;
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            instance->enqueue(RidBleEventType::CONFIGURED, param->adv_data_raw_cmpl.status);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            instance->enqueue(RidBleEventType::STARTED, param->adv_start_cmpl.status);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            instance->enqueue(RidBleEventType::STOPPED, param->adv_stop_cmpl.status);
            break;
        default:
            break;
    }
}
