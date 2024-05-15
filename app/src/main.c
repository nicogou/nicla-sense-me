#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include "bluetooth/ble_manager.h"
#include "bhi_common/common.h"
#include "bhi_euler/euler.h"
#include "bhi_acc_gyro/acc_gyro.h"
#include "bhi_klio/klio.h"
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/charger.h>
#include <app/drivers/bq25120a.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define NICLA_LED_DRIVER DT_ALIAS(is31fl3194)
static const struct device *led_driver_dev = DEVICE_DT_GET_ANY(issi_is31fl3194);

// #define NICLA_CHARGER DT_ALIAS(bq25120a)
static const struct device *charger_dev = DEVICE_DT_GET_ANY(ti_bq25120a);

#define BHI260AP_INT DT_ALIAS(bhi260apint)
static const struct gpio_dt_spec bhi_int = GPIO_DT_SPEC_GET_OR(BHI260AP_INT, gpios, {0});

#include "bhy2.h"
#include "bhy2_parse.h"

enum bhy2_intf intf;

int main(void)
{
	int8_t rslt;
	uint8_t boot_status;
	uint16_t version = 0;

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	if (!device_is_ready(led_driver_dev)) {
		LOG_ERR("LED driver not ready!");
	}

	if (!device_is_ready(charger_dev)) {
		LOG_ERR("Charger not ready!");
	}

	ble_manager_init();
	ble_manager_start_advertising();

	init_bhi260ap(bhy2_get_dev(), &intf);

	/* Check if the sensor is ready to load firmware */
	rslt = bhy2_get_boot_status(&boot_status, bhy2_get_dev());
	print_api_error(rslt, bhy2_get_dev(), __FILE__, __LINE__);
	if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
		upload_firmware(boot_status, bhy2_get_dev());

		rslt = bhy2_get_kernel_version(&version, bhy2_get_dev());
		print_api_error(rslt, bhy2_get_dev(), __FILE__, __LINE__);
		if ((rslt == BHY2_OK) && (version != 0)) {
			LOG_DBG("Boot successful. Kernel version %u.", version);
		}

		common_register_callback(bhy2_get_dev());
		acc_gyro_register_callback(bhy2_get_dev());
	} else {
		LOG_ERR("Host interface not ready. Exiting");
		return 0;
	}

	/* Update the callback table to enable parsing of sensor data */
	rslt = bhy2_update_virtual_sensor_list(bhy2_get_dev());
	print_api_error(rslt, bhy2_get_dev(), __FILE__, __LINE__);

	acc_gyro_cfg_virtual_sensor(bhy2_get_dev());

	while (rslt == BHY2_OK) {
		if (!gpio_pin_get_dt(&bhi_int)) {
			// common_fifo_process(bhy2_get_dev());
		}
	}

	return 0;
}
