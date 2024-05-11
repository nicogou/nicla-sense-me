#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include "bluetooth/ble_manager.h"
#include <zephyr/drivers/led.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define NICLA_LED_DRIVER DT_ALIAS(is31fl3194)
static const struct device *led_driver_dev = DEVICE_DT_GET_ANY(issi_is31fl3194);

int main(void)
{
	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	if (!device_is_ready(led_driver_dev)) {
		LOG_ERR("LED driver not ready!");
	}

	uint8_t color[3] = {0xd6, 0x84, 0x00};

	ble_manager_init();
	ble_manager_start_advertising();
	while (1) {
		led_set_color(led_driver_dev, 0, 3, color);
		led_on(led_driver_dev, 0);

		k_sleep(K_MSEC(500));
		led_off(led_driver_dev, 0);
		k_sleep(K_MSEC(500));
	}

	return 0;
}
