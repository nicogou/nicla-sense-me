#pragma once

#include "bhy2.h"
#include "bhy2_parse.h"
#include "bhi_common/common.h"
#include <zephyr/logging/log.h>

void parse_euler(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void euler_register_callback(struct bhy2_dev *dev);
void euler_cfg_virtual_sensor(struct bhy2_dev *dev);
