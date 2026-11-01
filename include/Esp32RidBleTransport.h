#pragma once

#include "RidBleBroadcaster.h"

#include <atomic>
#include <esp_gap_ble_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Sole owner of Bluedroid GAP, using ESP-IDF 4.4 legacy raw advertising APIs.
// No BLEDevice/GATT server, structured AD builder, or application NVS storage.
class Esp32RidBleTransport : public RidBleTransport
{
public:
    int32_t begin() override;
    int32_t configure(const uint8_t* data, size_t length) override;
    int32_t start() override;
    int32_t stop() override;
    bool poll(RidBleTransportEvent& event) override;
    const uint8_t* publicAddress() const { return address; }
    const char* initializationStep() const { return initStep; }

private:
    static Esp32RidBleTransport* instance;
    static constexpr size_t QUEUE_LENGTH = 8;
    StaticQueue_t queueControl = {};
    uint8_t queueStorage[QUEUE_LENGTH * sizeof(RidBleTransportEvent)] = {};
    QueueHandle_t events = nullptr;
    std::atomic<bool> overflow{false};
    uint8_t address[6] = {};
    esp_ble_adv_params_t params = {};
    const char* initStep = "not_started";
    static void gapCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
    void enqueue(RidBleEventType type, int32_t status);
};
