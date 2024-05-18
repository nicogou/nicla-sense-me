#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include "app/app.h"
#include "bluetooth/ble_manager.h"

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/charger.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *led_driver_dev = DEVICE_DT_GET_ANY(issi_is31fl3194);

static const struct device *charger_dev = DEVICE_DT_GET_ANY(ti_bq25120a);

int main(void)
{

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	if (!device_is_ready(led_driver_dev)) {
		LOG_ERR("LED driver not ready!");
	}

	if (!device_is_ready(charger_dev)) {
		LOG_ERR("Charger not ready!");
	}

	ble_manager_init();
	ble_manager_start_advertising();

	app_init();

	return app_run();
}
