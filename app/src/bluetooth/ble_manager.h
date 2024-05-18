#pragma once

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include "nicla_service.h"
#include "ble_nicla_control.h"
#include "nicla_zbus/nicla_zbus.h"

#define STACKSIZE 512 // CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY  7

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

typedef enum {
	BLE_DISCONNECTED,
	BLE_CONNECTED,
} ble_instruction_type_t;

uint8_t ble_manager_init();
uint32_t ble_manager_start_advertising();
uint32_t ble_manager_stop_advertising();
bool ble_manager_is_connected();
bool ble_manager_control_is_subscribed();
