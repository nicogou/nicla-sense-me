#pragma once

#include "bhy2.h"
#include "bhy2_klio.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <bluetooth/ble_nicla_control.h>
#include "bhi_klio/klio.h"

#define WORK_BUFFER_SIZE 2048
#define BHY2_RD_WR_LEN   256 /* MCU maximum read write length */
#define EULER_SENSOR_ID  BHY2_SENSOR_ID_ORI_WU

void init_bhi260ap(struct bhy2_dev *dev, enum bhy2_intf *intf);
struct bhy2_dev *bhy2_get_dev();
void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bhy2_delay_us(uint32_t period, void *intf_ptr);
void print_api_error(int8_t rslt, struct bhy2_dev *dev, char *file, int line);
char *get_sensor_error_text(uint8_t sensor_error);
char *get_sensor_name(uint8_t sensor_id);
char *get_klio_error(bhy2_klio_driver_error_state_t error);
