#pragma once

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>

#define NICLA_ZBUS_UNKNOWN_TYPE 255

// Instruction message type
typedef enum {
	INSTRUCTION_SOURCE_APP,
	INSTRUCTION_SOURCE_BLE,
	INSTRUCTION_SOURCE_CHARGER,
	INSTRUCTION_SOURCE_COUNT
} instruction_source_t;

typedef struct {
	instruction_source_t source;
	uint8_t type;
} instruction_msg_t;
