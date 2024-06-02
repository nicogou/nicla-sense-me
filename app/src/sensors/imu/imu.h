#pragma once

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "bhi_common/common.h"
#include "bhi_euler/euler.h"
#include "bhi_acc_gyro/acc_gyro.h"
#include "bhi_klio/klio.h"

#include "bhy2.h"
#include "bhy2_parse.h"

int imu_init();
int imu_start_acq();
void imu_start(float sample_freq, uint32_t latency);
void imu_stop();
