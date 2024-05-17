#pragma once

#include "bhy2.h"
#include "bhy2_parse.h"
#include "bhi_common/common.h"
#include <zephyr/logging/log.h>

void parse_acc(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parse_gyro(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void acc_gyro_register_callback(struct bhy2_dev *dev);
void acc_gyro_cfg_virtual_sensor(struct bhy2_dev *dev, float s_r, uint32_t l);
