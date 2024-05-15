#include "klio.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bhi_klio, CONFIG_APP_LOG_LEVEL);

K_SEM_DEFINE(bhy2_sem, 1, 1);
static bool updating = false;

static bool imu_started = false;
static bool learning_started = false;
static bool recognition_started = false;
static uint8_t imu_odr = 0;

bool klio_get_imu_started()
{
	return imu_started;
}

void klio_set_imu_started(bool b)
{
	imu_started = b;
}

uint8_t klio_get_imu_odr()
{
	return imu_odr;
}

void klio_set_imu_odr(uint8_t u)
{
	imu_odr = u;
}

bool klio_get_learning_started()
{
	return learning_started;
}

void klio_set_learning_started(bool b)
{
	learning_started = b;
}

bool klio_get_recognition_started()
{
	return recognition_started;
}

void klio_set_recognition_started(bool b)
{
	recognition_started = b;
}

bool klio_get_updating()
{
	return updating;
}

void klio_set_updating(bool u)
{
	updating = u;
}

void klio_imu_state_update(uint8_t data_type, uint8_t data)
{
	uint8_t ble_data[2];
	ble_data[0] = data_type;
	ble_data[1] = data;
	ble_nicla_control_queue(NICLA_CONTROL_SYS_IMU_STATE_CHANGED, ble_data, sizeof(ble_data));
}

static klio_runtime_t klio_rt;
/* Example pattern, BHI260 should be pointing upward, held level, and moved up and down at
 * about 1.5Hz */
static uint8_t klio_example_pattern_id = 0;
static uint8_t klio_example_pattern[] = {

	/* Original pattern
	0x52, 0x42, 0x31, 0x06, 0x03, 0x8b, 0xff, 0x3c, 0x40, 0x0a, 0xd7, 0x23, 0x3c, 0x73, 0xfe,
	0xa7, 0xc0, 0x38, 0x44, 0xbd, 0x40, 0xbb, 0x1f, 0xe5, 0x3e, 0x38, 0x6f, 0x30, 0x3f, 0x50,
	0x89, 0x74, 0x3f, 0x4d, 0x2a, 0xf8, 0x3c, 0x45, 0x61, 0xd9, 0x40, 0x6d, 0x21, 0x7f, 0x40,
	0xd0, 0x80, 0x8f, 0x3d, 0x9e, 0x39, 0x33, 0xbd, 0x51, 0xc5, 0x0e, 0x3f, 0x64, 0x94, 0x80,
	0x3c, 0xba, 0x90, 0xd2, 0x3e, 0xf8, 0xd8, 0x37, 0xbc, 0xed, 0x50, 0xea, 0x3d, 0xf4, 0x61,
	0x16, 0x3f, 0x75, 0xc9, 0x9b, 0xbe, 0x24, 0x20, 0xf7, 0x3c, 0x91, 0x16, 0x5b, 0xbd, 0x0f,
	    0x61, 0x21, 0xbc, 0x23, 0xce, 0x80, 0x3e, 0x46, 0x8c, 0x93, 0x3d, 0x0c, 0x70, 0x16,
	0x3e, 0x02, 0xf9, 0x9b, 0x3a, 0x12, 0x48, 0xbc, 0x3d, 0x2e, 0x1f, 0xba, 0x3d, 0xe9, 0x82,
	0xf5, 0xbe, 0xb4, 0xbd, 0xe8, 0x3d, 0xc6, 0x79, 0x02, 0xbd, 0x8a, 0x1a, 0x00, 0x3b, 0x87,
	0x22, 0x81, 0x3e, 0x96, 0x57, 0x05, 0x3e, 0xcb, 0x03, 0xcb, 0xbf, 0x34, 0x2d, 0x93, 0x3e,
	0x26, 0x6c, 0xff, 0xbd, 0x52, 0xb0, 0x84, 0x3b
	*/

	0x52, 0x42, 0x31, 0x06, 0x03, 0xa2, 0x38, 0xe7, 0x40, 0x0a, 0xd7, 0x23, 0x3c, 0x53, 0x0c,
	0xa5, 0x3d, 0x21, 0x60, 0xf5, 0xbe, 0x10, 0x53, 0xc3, 0x3e, 0xc5, 0x39, 0x43, 0xbd, 0xe9,
	0x05, 0xf5, 0xbd, 0xe8, 0xa8, 0x24, 0x3d, 0xd1, 0xc4, 0x44, 0xc0, 0xa7, 0xd8, 0x96, 0x40,
	0x9a, 0xfe, 0xa0, 0xc0, 0x88, 0xc2, 0xbe, 0x3e, 0x8f, 0x37, 0x3a, 0xbe, 0x33, 0x48, 0xcd,
	0x3d, 0x5d, 0xa8, 0xc4, 0x40, 0x6e, 0xda, 0x84, 0x40, 0x0c, 0x07, 0x92, 0xc0, 0xcc, 0x78,
	0x0b, 0x3f, 0xe5, 0x81, 0xbf, 0x3f, 0x7e, 0xbc, 0xfe, 0x3d, 0xd5, 0xbb, 0x74, 0x3f, 0x57,
	0x28, 0x96, 0xc0, 0xec, 0xc2, 0x9a, 0xc0, 0xf4, 0x35, 0x5a, 0x3f, 0x69, 0x9e, 0xb9, 0xbd,
	0x75, 0xbe, 0x84, 0x3e, 0x2e, 0xc0, 0xad, 0x3c, 0x3f, 0x26, 0xcc, 0x3d, 0xdc, 0x9d, 0x0c,
	0xbd, 0x75, 0x66, 0xbc, 0xbd, 0x96, 0xa5, 0xe8, 0x3c, 0x15, 0x57, 0x3c, 0x3b, 0xe8, 0x12,
	0xea, 0x3c, 0x71, 0x87, 0x76, 0xbe, 0xd0, 0x96, 0xaf, 0xbe, 0xa3, 0x64, 0x4a, 0x3e, 0xe4,
	0xdd, 0xc1, 0xbd, 0x9f, 0x52, 0x52, 0x3b};

static klio_pattern_t *klio_patterns_runtime;

void klio_init_klio_rt(struct bhy2_dev *dev)
{

	klio_rt.bhy2 = bhy2_get_dev();
	/* sensor_state will be set by parameter write */
	klio_rt.sensor_state.learning_enabled = 0;
	klio_rt.sensor_state.learning_reset = 0;
	klio_rt.sensor_state.recognition_enabled = 0;
	klio_rt.sensor_state.recognition_reset = 0;
	klio_rt.max_patterns = 0;                  /* Will be retrieved by parameter read */
	klio_rt.max_pattern_size = 0;              /* Will be retrieved by parameter read */
	klio_rt.ignore_insignificant_movement = 1; /* Will be set by parameter write */
	klio_rt.pattern_write_back_index = 0;      /* Used by callback routine */
	klio_rt.similarity_result_buf =
		NULL; /* Will be allocated after we know how many patterns are supported */
	klio_rt.similarity_idx_buf =
		NULL; /* Will be allocated after we know how many patterns are supported */
}

void klio_enable_disable_learning(struct bhy2_dev *dev, bool enable, bool reset)
{
	/* sensor_state will be set by parameter write */
	klio_rt.sensor_state.learning_enabled = enable;
	klio_rt.sensor_state.learning_reset = reset;
	updating = true;
}

void klio_enable_disable_recognition(struct bhy2_dev *dev, bool enable, bool reset)
{
	/* sensor_state will be set by parameter write */
	klio_rt.sensor_state.recognition_enabled = enable;
	klio_rt.sensor_state.recognition_reset = reset;
	updating = true;
}

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "bhi260ap_klio/BHI260AP_klio_turbo-flash.fw.h"
#endif

void klio_register_callback(struct bhy2_dev *dev)
{
	int8_t rslt;

	rslt = bhy2_register_fifo_parse_callback(KLIO_SENSOR_ID, parse_klio, (void *)&klio_rt, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_get_and_process_fifo(common_get_work_buffer(), WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void klio_get_klio_state(struct bhy2_dev *dev)
{
	int8_t rslt;
	bhy2_klio_sensor_state_t tmp_state;
	rslt = bhy2_klio_get_state(&tmp_state, dev);
	LOG_HEXDUMP_DBG(&tmp_state, sizeof(tmp_state), "Current sensor state:");
	klio_rt.sensor_state = tmp_state;
}

uint8_t klio_get_max_pattern_size()
{
	if (klio_rt.max_pattern_size == 0) {
		LOG_ERR("Max pattern size not retrieved from sensor (%u)",
			klio_rt.max_pattern_size);
	}
	return klio_rt.max_pattern_size;
}

void klio_cfg_virtual_sensor(struct bhy2_dev *dev, bool start_learning, bool start_recognition)
{
	int8_t rslt;
	/* Get number of supported patterns */
	uint8_t param_buf[PARAM_BUF_LEN];
	uint16_t length = sizeof(param_buf);

	bool init_patterns_runtime = true;
	if (klio_rt.max_patterns != 0) {
		init_patterns_runtime = false;
	}

	if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
		// We can't take the semaphore, device is not ready.
		LOG_ERR("Device not ready to receive commands, exiting");
		return;
	}
	rslt = bhy2_klio_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, param_buf, &length,
				       dev);
	k_sem_give(&bhy2_sem);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	print_klio_status(dev, __FILE__, __LINE__);
	klio_rt.max_patterns = *((uint16_t *)param_buf);
	klio_rt.similarity_result_buf = malloc(sizeof(float) * klio_rt.max_patterns);
	klio_rt.similarity_idx_buf = malloc(sizeof(uint8_t) * klio_rt.max_patterns);

	if (init_patterns_runtime) {
		klio_patterns_runtime = calloc(klio_rt.max_patterns, sizeof(klio_pattern_t));
	}

	if (klio_rt.similarity_result_buf == NULL || klio_rt.similarity_idx_buf == NULL ||
	    klio_patterns_runtime == NULL) {
		LOG_ERR("Unable to allocate Klio buffers. Exiting");

		while (true) {
			/* loop indefinitely */
		}
	}

	/* Get maximum supported pattern size */
	length = sizeof(param_buf);
	if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
		// We can't take the semaphore, device is not ready.
		LOG_ERR("Device not ready to receive commands, exiting");
		return;
	}
	rslt = bhy2_klio_get_parameter(KLIO_PARAM_PATTERN_BLOB_SIZE, param_buf, &length, dev);
	k_sem_give(&bhy2_sem);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	print_klio_status(dev, __FILE__, __LINE__);
	klio_rt.max_pattern_size = *((uint16_t *)param_buf);
	LOG_DBG("KLIO_PARAM_PATTERN_BLOB_SIZE %u", klio_rt.max_pattern_size);

	/* Set klio state (learning/recognition enable/disable and reset) */
	if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
		// We can't take the semaphore, device is not ready.
		LOG_ERR("Device not ready to receive commands, exiting");
		return;
	}
	rslt = bhy2_klio_set_state(&klio_rt.sensor_state, dev);
	k_sem_give(&bhy2_sem);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	print_klio_status(dev, __FILE__, __LINE__);

	if (start_learning) {
		/* Prevent learning with small movements, parameter writes should be done after
		 * reset and before sensor enable */
		if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
			// We can't take the semaphore, device is not ready.
			LOG_ERR("Device not ready to receive commands, exiting");
			return;
		}
		rslt = bhy2_klio_set_parameter(KLIO_PARAM_LEARNING_IGNORE_INSIG_MOVEMENT,
					       &klio_rt.ignore_insignificant_movement,
					       sizeof(klio_rt.ignore_insignificant_movement), dev);
		k_sem_give(&bhy2_sem);
		print_api_error(rslt, dev, __FILE__, __LINE__);
		print_klio_status(dev, __FILE__, __LINE__);
	}

	/* Write example pattern */
	for (uint8_t ii = 0; ii < klio_rt.max_patterns; ii++) {
		if (klio_patterns_runtime[ii].pattern_size !=
		    0) { // If a pattern is saved, write it to the IMU and enable it.
			LOG_DBG("Writing pattern n°%u", ii);
			if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
				// We can't take the semaphore, device is not ready.
				LOG_ERR("Device not ready to receive commands, exiting");
				return;
			}
			rslt = bhy2_klio_write_pattern(ii,
						       klio_patterns_runtime[ii].pattern_parameters,
						       klio_patterns_runtime[ii].pattern_size, dev);
			k_sem_give(&bhy2_sem);
			print_api_error(rslt, dev, __FILE__, __LINE__);
			print_klio_status(dev, __FILE__, __LINE__);

			// Enables the pattern for recognition, enabling should be done after
			// writing the pattern
			if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
				// We can't take the semaphore, device is not ready.
				LOG_ERR("Device not ready to receive commands, exiting");
				return;
			}
			rslt = bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE, &ii, 1, dev);
			k_sem_give(&bhy2_sem);
			print_api_error(rslt, dev, __FILE__, __LINE__);
			print_klio_status(dev, __FILE__, __LINE__);
		}
	}

	float sample_rate = 25.0; /* Read out data measured at 25Hz */
	if (!klio_rt.sensor_state.learning_enabled && !klio_rt.sensor_state.recognition_enabled) {
		sample_rate = 0.0;
	}
	uint32_t report_latency_ms = 0; /* Report immediately */

	if (k_sem_take(&bhy2_sem, K_FOREVER) != 0) {
		// We can't take the semaphore, device is not ready.
		LOG_ERR("Device not ready to receive commands, exiting");
		return;
	}
	rslt = bhy2_set_virt_sensor_cfg(KLIO_SENSOR_ID, sample_rate, report_latency_ms, dev);
	k_sem_give(&bhy2_sem);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	LOG_INF("Enable %s at %.2f Hz.", get_sensor_name(KLIO_SENSOR_ID), (double)sample_rate);

	if (start_learning) {
		klio_set_learning_started(true);
	} else {
		klio_set_learning_started(false);
	}

	if (start_recognition) {
		klio_set_recognition_started(true);
	} else {
		klio_set_recognition_started(false);
	}

	if (!klio_get_imu_started() && ble_manager_is_connected()) {
		// Warn BLE peer that learning/recognition has started.
		// This is needed because the IMU changes state only when a KLIO event is detected.
		// When learning this happens reasonable fast, but in recognition, if no learnt
		// pattern is made, no events are published.
		klio_imu_state_update(IMU_STATE_LEARNING, klio_get_learning_started());
		klio_imu_state_update(IMU_STATE_RECOGNITION, klio_get_recognition_started());
	}
}

void klio_process(struct bhy2_dev *dev)
{
	int8_t rslt;
	/* Data from the FIFO is read and the relevant callbacks if registered are called */
	if (k_sem_take(&bhy2_sem, K_FOREVER)) { // Prevent another thread to talk to the sensor.
		LOG_ERR("process device not ready");
	}
	rslt = bhy2_get_and_process_fifo(common_get_work_buffer(), WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	k_sem_give(&bhy2_sem);
}

void klio_free()
{
	free(klio_rt.similarity_result_buf);
	free(klio_rt.similarity_idx_buf);
	// Free all saved patterns parameters.
	for (int ii = 0; ii < klio_rt.max_patterns; ii++) {
		if (klio_patterns_runtime->pattern_size != 0) {
			free(klio_patterns_runtime->pattern_parameters);
		}
	}
	free(klio_patterns_runtime);
}

void print_klio_error(bhy2_klio_driver_error_state_t status, char *file, int line)
{
	if (status != KLIO_DRIVER_ERROR_NONE) {
		LOG_ERR("%s - file %s @%i", get_klio_error(status), file, line);
		exit(0);
	}
}

void print_klio_status(struct bhy2_dev *bhy2, char *file, int line)
{
	uint32_t klio_status;
	int8_t rslt = bhy2_klio_read_reset_driver_status(&klio_status, bhy2);

	print_api_error(rslt, bhy2, __FILE__, __LINE__);
	print_klio_error((bhy2_klio_driver_error_state_t)klio_status, file, line);
}

void parse_klio(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	klio_runtime_t *klio_rt = (klio_runtime_t *)callback_ref;
	bhy2_klio_sensor_frame_t data;
	uint32_t s, ns;

	uint8_t recognition_data_to_send[5];
	uint8_t learning_data_to_send[2];

	if (callback_info->data_size != 11) /* Check for a valid payload size. Includes sensor ID */
	{
		LOG_ERR("FIFO data does not have the correct size");
		return;
	}

	uint64_t timestamp = *callback_info->time_stamp; /* Store the last timestamp */

	timestamp = timestamp * 15625; /* Timestamp is now in nanoseconds */
	s = (uint32_t)(timestamp / UINT64_C(1000000000));
	ns = (uint32_t)(timestamp - (s * UINT64_C(1000000000)));

	memcpy(&data, callback_info->data_ptr, sizeof(data));
	LOG_INF("SID: %u; T: %u.%09u; Learning [Id:%d Progress:%u Change:%u]; Recognition[Id:%d "
		"Count:%f]",
		callback_info->sensor_id, s, ns, data.learn.index, data.learn.progress,
		data.learn.change_reason, data.recognize.index, (double)data.recognize.count);

	if (klio_get_learning_started()) {
		// Meaning a pattern was recognized.
		memcpy(&learning_data_to_send[0], &data.learn.progress,
		       sizeof(data.learn.progress));
		memcpy(&learning_data_to_send[1], &data.learn.change_reason,
		       sizeof(data.learn.change_reason));
		ble_nicla_control_queue(NICLA_CONTROL_LEARNING_PROGRESS, learning_data_to_send,
					sizeof(learning_data_to_send));
	}

	if (klio_get_learning_started() &&
	    data.learn.index != -1) /* -1 means nothing was learnt. */
	{
		float highest_similarity_score =
			parse_klio_handle_learnt_pattern(callback_info, s, ns, klio_rt);
	}

	if (klio_get_recognition_started() && data.recognize.index != 255) {
		// Meaning a pattern was recognized.
		recognition_data_to_send[0] = data.recognize.index;
		memcpy(&recognition_data_to_send[1], &data.recognize.count,
		       sizeof(data.recognize.count));
		ble_nicla_control_queue(NICLA_CONTROL_RECOGNITION_COUNT, recognition_data_to_send,
					sizeof(recognition_data_to_send));
	}
}

float parse_klio_handle_learnt_pattern(const struct bhy2_fifo_parse_data_info *callback_info,
				       uint32_t s, uint32_t ns, klio_runtime_t *klio_rt)
{
	uint8_t tmp_buf[PARAM_BUF_LEN];
	uint16_t bufsize = sizeof(tmp_buf);
	struct bhy2_dev *bhy2 = klio_rt->bhy2;
	float highest_similarity_score = 0.f;

	/* Read out learnt pattern */
	int8_t rslt = bhy2_klio_read_pattern(0, tmp_buf, &bufsize, bhy2);

	print_api_error(rslt, bhy2, __FILE__, __LINE__);
	print_klio_status(bhy2, __FILE__, __LINE__);

	LOG_INF("SID: %u; T: %u.%09u; PATTERN LEARNT: ", callback_info->sensor_id, s, ns);
	LOG_HEXDUMP_INF(tmp_buf, bufsize, "");

	uint8_t ble_data[247]; // Max MTU size = 247 (negotiated at connection)
	uint8_t ble_data_index = 0;
	memcpy(&ble_data[ble_data_index], &bufsize,
	       sizeof(bufsize)); // Copy pattern param number to BLE frame.
	ble_data_index += sizeof(bufsize);
	memcpy(&ble_data[ble_data_index], tmp_buf,
	       bufsize); // Copy pattern parameters to BLE frame.
	ble_data_index += bufsize;

	/* Write back learnt pattern for recognition */
	if (klio_rt->pattern_write_back_index < klio_rt->max_patterns) {
		/* Write pattern for recognition, note that this resets recognition statistics (and
		 * repetition counts) */
		rslt = bhy2_klio_write_pattern(klio_rt->pattern_write_back_index, tmp_buf, bufsize,
					       bhy2);
		print_api_error(rslt, bhy2, __FILE__, __LINE__);
		print_klio_status(bhy2, __FILE__, __LINE__);

		if (klio_rt->pattern_write_back_index > 0) {
			/* Compare current pattern against all previously stored ones */
			for (uint8_t i = 0; i < klio_rt->pattern_write_back_index; i++) {
				klio_rt->similarity_idx_buf[i] = i;
			}

			rslt = bhy2_klio_similarity_score_multiple(
				klio_rt->pattern_write_back_index, klio_rt->similarity_idx_buf,
				klio_rt->pattern_write_back_index, klio_rt->similarity_result_buf,
				klio_rt->bhy2);
			print_api_error(rslt, bhy2, __FILE__, __LINE__);
			print_klio_status(bhy2, __FILE__, __LINE__);
			LOG_INF("SID: %u; T: %u.%09u; SIMILARITY SCORE TO ALREADY STORED "
				"PATTERNS: ",
				callback_info->sensor_id, s, ns);
			memcpy(&ble_data[ble_data_index], &klio_rt->pattern_write_back_index,
			       sizeof(klio_rt->pattern_write_back_index));
			ble_data_index += sizeof(klio_rt->pattern_write_back_index);
			for (uint8_t i = 0; i < klio_rt->pattern_write_back_index; i++) {
				// Add similarity score to BLE frame
				memcpy(&ble_data[ble_data_index],
				       &klio_rt->similarity_result_buf[i],
				       sizeof(klio_rt->similarity_result_buf[i]));
				ble_data_index += sizeof(klio_rt->similarity_result_buf);

				float tmp_score = klio_rt->similarity_result_buf[i];

				LOG_INF("%d: %f ", i, (double)tmp_score);

				if (tmp_score > highest_similarity_score) {
					highest_similarity_score = tmp_score;
				}
			}
		} else {
			// If there is only one saved pattern, don't send similarity with other
			// patterns.
			ble_data[ble_data_index] = (uint8_t)0;
			ble_data_index += sizeof(uint8_t);
		}
	}

	ble_nicla_control_queue(NICLA_CONTROL_LEARNING_PATTERN_LEARNT, ble_data, ble_data_index);

	/* If we have no stored patterns return will be 0.f */

	return highest_similarity_score;
}

void klio_save_pattern(struct bhy2_dev *dev, uint8_t *pattern, uint8_t pattern_size)
{
	LOG_DBG("Saving pattern of size %u at id %u", pattern_size,
		klio_rt.pattern_write_back_index);
	LOG_HEXDUMP_DBG(pattern, pattern_size, "parameters :");
	klio_patterns_runtime[klio_rt.pattern_write_back_index].pattern_size = pattern_size;
	klio_patterns_runtime[klio_rt.pattern_write_back_index].pattern_parameters =
		malloc(pattern_size * sizeof(uint8_t));

	if (klio_patterns_runtime[klio_rt.pattern_write_back_index].pattern_parameters == NULL) {
		LOG_ERR("Unable to allocate pattern buffers. Exiting");
		while (true) {
			/* loop indefinitely */
		}
	}

	memcpy(klio_patterns_runtime[klio_rt.pattern_write_back_index].pattern_parameters, pattern,
	       pattern_size);

	// Send pattern ID over BLE.
	ble_nicla_control_queue(RESPONSE_BIT | NICLA_CONTROL_LEARNING_SAVE_PATTERN,
				&klio_rt.pattern_write_back_index, 1);

	klio_rt.pattern_write_back_index++;
}
