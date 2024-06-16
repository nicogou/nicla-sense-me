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
	uint64_t wr_buffer[IMU_SAMPLE_RATE];
	uint64_t rd_buffer[IMU_SAMPLE_RATE];
} u64_buffer_t;

typedef struct {
	char* name;
	i16_buffer_t x;
	i16_buffer_t y;
	i16_buffer_t z;
	u64_buffer_t ts;
} imu_buffer_t;

void sensor_buffer_put(imu_buffer_t *buf, struct bhy2_data_xyz data, uint64_t timestamp);
void sensor_buffer_put_acc(struct bhy2_data_xyz data, uint64_t timestamp);
void sensor_buffer_put_gyro(struct bhy2_data_xyz data, uint64_t timestamp);
