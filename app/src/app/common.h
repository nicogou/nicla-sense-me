#pragma once

typedef enum {
	RECORDING_START,
	RECORDING_STOP,
} app_instruction_type_t;

/* List of app states */
enum app_state {
	IDLE,
	RECORDING,
};
