#include "imu.h"

LOG_MODULE_REGISTER(imu, CONFIG_APP_LOG_LEVEL);

#define BHI260AP_INT DT_ALIAS(bhi260apint)
static const struct gpio_dt_spec bhi_int = GPIO_DT_SPEC_GET_OR(BHI260AP_INT, gpios, {0});

static struct gpio_callback imu_cb_data;

static enum bhy2_intf intf;

void imu_start(float sample_freq, uint32_t latency)
{
	acc_gyro_cfg_virtual_sensor(bhy2_get_dev(), sample_freq, latency);
}

void imu_stop()
{
	imu_start(0.0, 0);
}

static void imu_int_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	common_fifo_process(bhy2_get_dev());
}

int imu_init()
{
	int8_t rslt;
	uint8_t boot_status;
	uint16_t version = 0;
	int ret;

	if (!gpio_is_ready_dt(&bhi_int)) {
		printk("Error: bhi_int device %s is not ready\n", bhi_int.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&bhi_int, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, bhi_int.port->name,
		       bhi_int.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&bhi_int, GPIO_INT_EDGE_TO_INACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret,
		       bhi_int.port->name, bhi_int.pin);
		return 0;
	}

	gpio_init_callback(&imu_cb_data, imu_int_cb, BIT(bhi_int.pin));
	gpio_add_callback(bhi_int.port, &imu_cb_data);

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
		return -1;
	}

	/* Update the callback table to enable parsing of sensor data */
	rslt = bhy2_update_virtual_sensor_list(bhy2_get_dev());
	print_api_error(rslt, bhy2_get_dev(), __FILE__, __LINE__);

	imu_stop();
	return 0;
}

static void imu_thread_run()
{
	imu_init();

	while (true) {
		k_msleep(1);
	}
}

K_THREAD_DEFINE(imu_thread, 2048, imu_thread_run, NULL, NULL, NULL, 5, 0, 0);
