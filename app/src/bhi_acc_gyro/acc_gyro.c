#include "acc_gyro.h"

LOG_MODULE_REGISTER(bhi_acc_gyro, CONFIG_APP_LOG_LEVEL);

static uint8_t work_buffer[WORK_BUFFER_SIZE];
static uint8_t accuracy[2]; /* Accuracy is reported as a meta event. It is being
					       printed alongside the data */

void acc_gyro_register_callback(struct bhy2_dev *dev)
{
	int8_t rslt;

	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, accuracy,
						 dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event,
						 accuracy, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	rslt = bhy2_register_fifo_parse_callback(ACC_SENSOR_ID, parse_acc, accuracy, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	rslt = bhy2_register_fifo_parse_callback(GYRO_SENSOR_ID, parse_gyro, accuracy, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void acc_gyro_cfg_virtual_sensor(struct bhy2_dev *dev)
{
	int8_t rslt;
	float sample_rate = 100.0;      /* Read out data measured at 100Hz */
	uint32_t report_latency_ms = 0; /* Report immediately */
	rslt = bhy2_set_virt_sensor_cfg(ACC_SENSOR_ID, sample_rate, report_latency_ms, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	LOG_INF("Enable %s at %.2fHz.", get_sensor_name(ACC_SENSOR_ID), (double)sample_rate);
	rslt = bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, sample_rate, report_latency_ms, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	LOG_INF("Enable %s at %.2fHz.", get_sensor_name(GYRO_SENSOR_ID), (double)sample_rate);
}

void acc_gyro_process(struct bhy2_dev *dev)
{
	int8_t rslt;
	/* Data from the FIFO is read and the relevant callbacks if registered are called */
	rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void parse_acc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	(void)callback_ref;
	struct bhy2_data_xyz data;
	uint32_t s, ns;
	if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
	{
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	LOG_DBG("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; acc: %u", callback_info->sensor_id, s,
		ns, (double)((float)data.x / 4096.0f), (double)((float)data.y / 4096.0f),
		(double)((float)data.z / 4096.0f), accuracy[0]);
}

void parse_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	(void)callback_ref;
	struct bhy2_data_xyz data;
	uint32_t s, ns;
	if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
	{
		return;
	}

	bhy2_parse_xyz(callback_info->data_ptr, &data);

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	LOG_DBG("SID: %u; T: %u.%09u; x: %f, y: %f, z: %f; acc: %u", callback_info->sensor_id, s,
		ns,
		(double)((float)data.x * (2 * 3.141592653589793f / 360.0f) / (32768.0f / 2000.0f)),
		(double)((float)data.y * (2 * 3.141592653589793f / 360.0f) / (32768.0f / 2000.0f)),
		(double)((float)data.z * (2 * 3.141592653589793f / 360.0f) / (32768.0f / 2000.0f)),
		accuracy[1]);
}