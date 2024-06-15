#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>
#include "app/app.h"
#include "bluetooth/ble_manager.h"
#include "nicla_sd/nicla_sd.h"

#include <zephyr/drivers/led.h>
#include <zephyr/drivers/charger.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct device *led_driver_dev = DEVICE_DT_GET_ANY(issi_is31fl3194);

static const struct device *charger_dev = DEVICE_DT_GET_ANY(ti_bq25120a);

#define USER_BUTTON DT_ALIAS(sw0)
static const struct gpio_dt_spec user_button =
	GPIO_DT_SPEC_GET_OR(USER_BUTTON, gpios, {0});

static struct gpio_callback user_button_cb_data;

void user_button_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	LOG_DBG("User button pressed at %" PRIu32, k_cycle_get_32());
}

int main(void)
{

	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	    if (!gpio_is_ready_dt(&user_button))
    {
        LOG_ERR("Error: user_button not ready");
    }

    int ret = gpio_pin_configure_dt(&user_button, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("Error %i: failed to configure user_button", ret);
    }

    ret = gpio_pin_interrupt_configure_dt(&user_button,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, user_button.port->name, user_button.pin);
		return 0;
	}

	gpio_init_callback(&user_button_cb_data, user_button_pressed, BIT(user_button.pin));
	gpio_add_callback(user_button.port, &user_button_cb_data);

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
