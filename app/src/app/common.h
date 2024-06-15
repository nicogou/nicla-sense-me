#pragma once

typedef enum {
	RECORDING_START,
	RECORDING_STOP,
	RECORDING_GO_TO_IDLE,
} app_instruction_type_t;

/* List of app states */
enum app_state {
	IDLE,
	RECORDING,
};
