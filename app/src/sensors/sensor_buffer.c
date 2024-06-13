#include "sensor_buffer.h"
#include "bhy2_defs.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(sensor_buffer, CONFIG_APP_LOG_LEVEL);

static imu_buffer_t acc_buffer = {.wr_idx = 0, .name = "Accel"};
static imu_buffer_t gyro_buffer = {.wr_idx = 0, .name = "Gyro"};

void sensor_buffer_put(imu_buffer_t *buf, struct bhy2_data_xyz data, uint64_t timestamp){
	buf->data[0][buf->wr_idx] = data.x;
	buf->data[1][buf->wr_idx] = data.y;
	buf->data[2][buf->wr_idx] = data.z;
	buf->timestamps[buf->wr_idx] = timestamp;
	buf->wr_idx++;

	if (buf->wr_idx == IMU_SAMPLE_RATE){
		LOG_WRN("%s buffer full, resetting wr_idx", buf->name);
		buf->wr_idx = 0;
	}
}

void sensor_buffer_put_acc(struct bhy2_data_xyz data, uint64_t timestamp){
	sensor_buffer_put(&acc_buffer, data, timestamp);
}

void sensor_buffer_put_gyro(struct bhy2_data_xyz data, uint64_t timestamp){
	sensor_buffer_put(&gyro_buffer, data, timestamp);
}
