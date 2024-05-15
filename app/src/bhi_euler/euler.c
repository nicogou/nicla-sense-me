#include "euler.h"

LOG_MODULE_REGISTER(bhi_euler, CONFIG_APP_LOG_LEVEL);

void euler_register_callback(struct bhy2_dev *dev)
{
	int8_t rslt;

	rslt = bhy2_register_fifo_parse_callback(EULER_SENSOR_ID, parse_euler,
						 common_get_accuracy(), dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_get_and_process_fifo(common_get_work_buffer(), WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void euler_cfg_virtual_sensor(struct bhy2_dev *dev)
{
	int8_t rslt;
	float sample_rate = 100.0;      /* Read out data measured at 50Hz */
	uint32_t report_latency_ms = 0; /* Report immediately */
	rslt = bhy2_set_virt_sensor_cfg(EULER_SENSOR_ID, sample_rate, report_latency_ms, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	LOG_INF("Enable %s at %.2fHz.", get_sensor_name(EULER_SENSOR_ID), (double)sample_rate);
}

void euler_process(struct bhy2_dev *dev)
{
	int8_t rslt;
	/* Data from the FIFO is read and the relevant callbacks if registered are called */
	rslt = bhy2_get_and_process_fifo(common_get_work_buffer(), WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	(void)callback_ref;
	struct bhy2_data_orientation data;
	uint32_t s, ns;
	if (callback_info->data_size != 7) /* Check for a valid payload size. Includes sensor ID */
	{
		return;
	}

	bhy2_parse_orientation(callback_info->data_ptr, &data);

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	LOG_DBG("SID: %u; T: %u.%09u; h: %f, p: %f, r: %f; acc: %u", callback_info->sensor_id, s,
		ns, (double)(data.heading * 360.0f / 32768.0f),
		(double)(data.pitch * 360.0f / 32768.0f), (double)(data.roll * 360.0f / 32768.0f),
		common_get_accuracy()[2]);
}
