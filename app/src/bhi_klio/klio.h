#pragma once

#include "bhy2.h"
#include "bhy2_parse.h"
#include "bhy2_klio.h"
#include "bhi_common/common.h"
#include <bluetooth/ble_nicla_control.h>

#define KLIO_SENSOR_ID BHY2_SENSOR_ID_KLIO
#define PARAM_BUF_LEN  252

typedef struct klio_runtime {
	struct bhy2_dev *bhy2;
	bhy2_klio_sensor_state_t sensor_state;
	uint16_t max_patterns;
	uint16_t max_pattern_size;
	uint8_t ignore_insignificant_movement;
	uint8_t pattern_write_back_index;
	float *similarity_result_buf;
	uint8_t *similarity_idx_buf;
} klio_runtime_t;

typedef struct __attribute__((packed)) klio_pattern {
	uint8_t pattern_size;
	uint8_t *pattern_parameters;
} klio_pattern_t;

struct bhy2_dev *klio_get_dev();
void klio_init_klio_rt(struct bhy2_dev *dev);
void klio_register_callback(struct bhy2_dev *dev);
void klio_cfg_virtual_sensor(struct bhy2_dev *dev, bool start_learning, bool start_recognition);
void klio_process(struct bhy2_dev *dev);
void klio_free();
void print_klio_error(bhy2_klio_driver_error_state_t status, char *file, int line);
void print_klio_status(struct bhy2_dev *bhy2, char *file, int line);
void parse_klio(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
float parse_klio_handle_learnt_pattern(const struct bhy2_fifo_parse_data_info *callback_info,
				       uint32_t s, uint32_t ns, klio_runtime_t *klio_rt);
void klio_enable_disable_learning(struct bhy2_dev *dev, bool enable, bool reset);
void klio_enable_disable_recognition(struct bhy2_dev *dev, bool enable, bool reset);
void klio_get_klio_state(struct bhy2_dev *dev);
void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev);

bool klio_get_imu_started();
void klio_set_imu_started(bool b);
bool klio_get_learning_started();
void klio_set_learning_started(bool b);
bool klio_get_recognition_started();
void klio_set_recognition_started(bool b);
uint8_t klio_get_imu_odr();
void klio_set_imu_odr(uint8_t u);
uint8_t klio_get_max_pattern_size();

void klio_imu_state_update(uint8_t data_type, uint8_t data);

void klio_save_pattern(struct bhy2_dev *dev, uint8_t *pattern, uint8_t pattern_size);
