#pragma once
#include "app/app.h"
#include "bhy2_defs.h"

typedef struct {
	uint8_t wr_idx;
	uint8_t rd_idx;
	int16_t wr_buffer[IMU_SAMPLE_RATE];
	int16_t rd_buffer[IMU_SAMPLE_RATE];
} i16_buffer_t;

typedef struct {
	uint8_t wr_idx;
	uint8_t rd_idx;
	uint32_t wr_buffer[IMU_SAMPLE_RATE];
	uint32_t rd_buffer[IMU_SAMPLE_RATE];
} u32_buffer_t;

typedef struct {
	char* name;
	i16_buffer_t x;
	i16_buffer_t y;
	i16_buffer_t z;
	u32_buffer_t ts;
	struct k_work work;
} imu_buffer_t;

void sensor_buffer_put(imu_buffer_t *buf, struct bhy2_data_xyz data, uint32_t timestamp);
void sensor_buffer_put_acc(struct bhy2_data_xyz data, uint32_t timestamp);
void sensor_buffer_put_gyro(struct bhy2_data_xyz data, uint32_t timestamp);
void sensor_buffer_init();
