#include "sensor_buffer.h"
#include "bhy2_defs.h"
#include "zephyr/logging/log.h"
#include "nicla_sd/nicla_sd.h"

LOG_MODULE_REGISTER(sensor_buffer, CONFIG_APP_LOG_LEVEL);

static imu_buffer_t acc_buffer = {.name = SESSION_ACC_FILE_NAME, .x = {.wr_idx = 0, .rd_idx = 0}, .y = {.wr_idx = 0, .rd_idx = 0}, .z = {.wr_idx = 0, .rd_idx = 0}, .ts = {.wr_idx = 0, .rd_idx = 0}};
static imu_buffer_t gyro_buffer = {.name = SESSION_GYRO_FILE_NAME, .x = {.wr_idx = 0, .rd_idx = 0}, .y = {.wr_idx = 0, .rd_idx = 0}, .z = {.wr_idx = 0, .rd_idx = 0}, .ts = {.wr_idx = 0, .rd_idx = 0}};

void sensor_buffer_put(imu_buffer_t *buf, struct bhy2_data_xyz data, uint64_t timestamp){
	buf->x.wr_buffer[buf->x.wr_idx++] = data.x;
	buf->y.wr_buffer[buf->y.wr_idx++] = data.y;
	buf->z.wr_buffer[buf->z.wr_idx++] = data.z;
	buf->ts.wr_buffer[buf->ts.wr_idx++] = timestamp;

	if (buf->ts.wr_idx == buf->x.wr_idx && buf->ts.wr_idx == buf->y.wr_idx && buf->ts.wr_idx == buf->z.wr_idx) {
		if (buf->ts.wr_idx == IMU_SAMPLE_RATE) {
			LOG_WRN("%s buffer full, resetting wr_idx and copying data to rd_buffer", buf->name);
			buf->ts.wr_idx = 0;
			buf->x.wr_idx = 0;
			buf->y.wr_idx = 0;
			buf->z.wr_idx = 0;
			memcpy(buf->ts.rd_buffer, buf->ts.wr_buffer, IMU_SAMPLE_RATE * sizeof(uint64_t));
			memcpy(buf->x.rd_buffer, buf->x.wr_buffer, IMU_SAMPLE_RATE * sizeof(int16_t));
			memcpy(buf->y.rd_buffer, buf->y.wr_buffer, IMU_SAMPLE_RATE * sizeof(int16_t));
			memcpy(buf->z.rd_buffer, buf->z.wr_buffer, IMU_SAMPLE_RATE * sizeof(int16_t));
			k_work_submit(&buf->work);
		}
	} else {
		LOG_WRN("Error incrementing write indexes");
	}
}

void sensor_buffer_put_acc(struct bhy2_data_xyz data, uint64_t timestamp){
	sensor_buffer_put(&acc_buffer, data, timestamp);
}

void sensor_buffer_put_gyro(struct bhy2_data_xyz data, uint64_t timestamp){
	sensor_buffer_put(&gyro_buffer, data, timestamp);
}

static void fifo_full_work_handler(struct k_work *item){
	imu_buffer_t *buf =
        CONTAINER_OF(item, imu_buffer_t, work);

	int16_t data[3];
	uint64_t ts;
	for (int ii = 0; ii < IMU_SAMPLE_RATE; ii++){
		data[0] = buf->x.rd_buffer[buf->x.rd_idx++];
		data[1] = buf->y.rd_buffer[buf->y.rd_idx++];
		data[2] = buf->z.rd_buffer[buf->z.rd_idx++];
		ts = buf->ts.rd_buffer[buf->ts.rd_idx++];
		nicla_sd_write(buf->name, data, sizeof(data), ts);
	}
	buf->x.rd_idx = 0;
	buf->y.rd_idx = 0;
	buf->z.rd_idx = 0;
	buf->ts.rd_idx = 0;
}

void sensor_buffer_init(){
	k_work_init(&acc_buffer.work, fifo_full_work_handler);
	k_work_init(&gyro_buffer.work, fifo_full_work_handler);
}
