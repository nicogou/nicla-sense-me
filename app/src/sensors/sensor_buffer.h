#pragma once
#include "app/app.h"
#include "bhy2_defs.h"

typedef struct {
	uint8_t wr_idx;
	int16_t data[3][IMU_SAMPLE_RATE]; // id 0 = x, id 1 = y, id 2 = z.
	char* name;
} imu_buffer_t;

void sensor_buffer_put(imu_buffer_t *buf, struct bhy2_data_xyz data);
void sensor_buffer_put_acc(struct bhy2_data_xyz data);
void sensor_buffer_put_gyro(struct bhy2_data_xyz data);
