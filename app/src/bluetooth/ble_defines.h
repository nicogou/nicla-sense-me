#pragma once

typedef enum {
	IMU_STATE_POWER_MODE,
	IMU_STATE_ODR,
	IMU_STATE_LEARNING,
	IMU_STATE_RECOGNITION,
} imu_state_t;

// Control communication types
#define NICLA_CONTROL_DATA_MAX_LEN 23
#define RESPONSE_BIT               0x80

#define COMMAND_CONFIRMATION 0x01

//////// Device System Commands
#define NICLA_CONTROL_SYS_SERIAL              0x01 // uint32_t deviceID : Serial Number
#define NICLA_CONTROL_SYS_RESREAS             0x02 // reset_reas_t : Reason for last reset
#define NICLA_CONTROL_SYS_RESET_TO_UF2        0x03 // Reset device into UF2 mode
#define NICLA_CONTROL_SYS_RESET               0x10 // Reset Device
#define NICLA_CONTROL_SYS_IMU_STATE_CHANGED   0x11 // Warns of a change in the IMU state
#define NICLA_CONTROL_SYS_SET_GET_RTC_TIME    0x12 // Get/Set RTC time
/////// Learning Commands
#define NICLA_CONTROL_LEARNING_START          0x20 // Start/stop KLIO learning
#define NICLA_CONTROL_LEARNING_PROGRESS       0x21
#define NICLA_CONTROL_LEARNING_PATTERN_LEARNT 0x22
#define NICLA_CONTROL_LEARNING_SAVE_PATTERN   0x23
/////// Recognition Commands
#define NICLA_CONTROL_RECOGNITION_START       0x30 // Start/stop recognition
#define NICLA_CONTROL_RECOGNITION_COUNT       0x31
/////// Recording Commands
#define NICLA_CONTROL_RECORDING_START_STOP    0x40 // Start/stop recording
///// Device BLE Commands
#define NICLA_CONTROL_BLE_DISCONN             0x60 // Force a disconnect from device side
