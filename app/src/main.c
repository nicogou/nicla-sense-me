#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <app_version.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

int main(void)
{
	LOG_INF("Zephyr Example Application %s", APP_VERSION_STRING);

	while (1) {
		k_sleep(K_MSEC(100));
	}

	return 0;
}
