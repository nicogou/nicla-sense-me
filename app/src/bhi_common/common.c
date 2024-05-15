#include "common.h"

LOG_MODULE_REGISTER(bhi_common, CONFIG_APP_LOG_LEVEL);

#define SPI_BHI260AP DT_NODELABEL(spi2)
// SPI configuration structures
const struct device *bmi_spi = DEVICE_DT_GET(SPI_BHI260AP);
const struct spi_config bmi_spi_cfg = {
	.frequency = DT_PROP(SPI_BHI260AP, clock_frequency),
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	.cs = SPI_CS_CONTROL_INIT(DT_NODELABEL(bhi260ap), 10),
};

static struct bhy2_dev bhy2;

static uint8_t work_buffer[WORK_BUFFER_SIZE];
static uint8_t accuracy[3]; /* Accuracy is reported as a meta event. It is being printed alongside
			       the data.
				   accuracy[0] = accelerometer accuracy.
				   accuracy[1] = gyro accuracy
				   accuracy[2] = euler accuracy
				    */

struct bhy2_dev *bhy2_get_dev()
{
	return &bhy2;
}

void init_bhi260ap(struct bhy2_dev *dev, enum bhy2_intf *intf)
{
	uint8_t product_id = 0;
	int8_t rslt;
	uint8_t hintr_ctrl, hif_ctrl;

	*intf = BHY2_SPI_INTERFACE;

	rslt = bhy2_init(BHY2_SPI_INTERFACE, bhy2_spi_read, bhy2_spi_write, bhy2_delay_us,
			 BHY2_RD_WR_LEN, NULL, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_soft_reset(dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_get_product_id(&product_id, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	/* Check for a valid product ID */
	if (product_id != BHY2_PRODUCT_ID) {
		LOG_WRN("Product ID read %X. Expected %X", product_id, BHY2_PRODUCT_ID);
	} else {
		LOG_INF("BHI260/BHA260 found. Product ID read %X", product_id);
	}

	/* Check the interrupt pin and FIFO configurations. Disable status and debug */
	hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;

	rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	rslt = bhy2_get_host_interrupt_ctrl(&hintr_ctrl, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	LOG_INF("Host interrupt control");
	LOG_INF("    Wake up FIFO %s.",
		(hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
	LOG_INF("    Non wake up FIFO %s.",
		(hintr_ctrl & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
	LOG_INF("    Status FIFO %s.",
		(hintr_ctrl & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
	LOG_INF("    Debugging %s.",
		(hintr_ctrl & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
	LOG_INF("    Fault %s.", (hintr_ctrl & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
	LOG_INF("    Interrupt is %s.",
		(hintr_ctrl & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
	LOG_INF("    Interrupt is %s triggered.",
		(hintr_ctrl & BHY2_ICTL_EDGE) ? "pulse" : "level");
	LOG_INF("    Interrupt pin drive is %s.",
		(hintr_ctrl & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");

	/* Configure the host interface */
	hif_ctrl = 0;
	rslt = bhy2_set_host_intf_ctrl(hif_ctrl, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

void common_register_callback(struct bhy2_dev *dev)
{
	int8_t rslt;

	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, accuracy,
						 dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
	rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, parse_meta_event,
						 accuracy, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);

	rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

uint8_t *common_get_accuracy()
{
	return accuracy;
}

uint8_t *common_get_work_buffer()
{
	return work_buffer;
}

void common_fifo_process(struct bhy2_dev *dev)
{
	int8_t rslt;
	/* Data from the FIFO is read and the relevant callbacks if registered are called */
	rslt = bhy2_get_and_process_fifo(work_buffer, WORK_BUFFER_SIZE, dev);
	print_api_error(rslt, dev, __FILE__, __LINE__);
}

// #define UPLOAD_FIRMWARE_TO_FLASH

#ifdef UPLOAD_FIRMWARE_TO_FLASH
#include "firmware/bhi260ap/BHI260AP_aux_BMM150-flash.fw.h"
#endif

void upload_firmware(uint8_t boot_stat, struct bhy2_dev *dev)
{
	uint8_t sensor_error;
	int8_t temp_rslt;
	int8_t rslt = BHY2_OK;

#ifdef UPLOAD_FIRMWARE_TO_FLASH
	if (boot_stat & BHY2_BST_FLASH_DETECTED) {
		uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
		uint32_t end_addr = start_addr + sizeof(bhy2_firmware_image);
		LOG_INF("Flash detected. Erasing flash to upload firmware");

		rslt = bhy2_erase_flash(start_addr, end_addr, dev);
		print_api_error(rslt, dev, __FILE__, __LINE__);
	} else {
		LOG_ERR("Flash not detected");

		rslt = BHY2_E_IO;
		print_api_error(rslt, dev, __FILE__, __LINE__);
	}

	LOG_INF("Loading firmware into FLASH.");
	rslt = bhy2_upload_firmware_to_flash(bhy2_firmware_image, sizeof(bhy2_firmware_image), dev);

	temp_rslt = bhy2_get_error_value(&sensor_error, dev);
	if (sensor_error) {
		LOG_ERR("%s", get_sensor_error_text(sensor_error));
	}

	print_api_error(rslt, dev, __FILE__, __LINE__);
	print_api_error(temp_rslt, dev, __FILE__, __LINE__);

#endif
	LOG_INF("Booting from FLASH.");
	rslt = bhy2_boot_from_flash(dev);

	temp_rslt = bhy2_get_error_value(&sensor_error, dev);
	if (sensor_error) {
		LOG_ERR("%s", get_sensor_error_text(sensor_error));
	}

	print_api_error(rslt, dev, __FILE__, __LINE__);
	print_api_error(temp_rslt, dev, __FILE__, __LINE__);
}

void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
	(void)callback_ref;
	uint8_t meta_event_type = callback_info->data_ptr[0];
	uint8_t byte1 = callback_info->data_ptr[1];
	uint8_t byte2 = callback_info->data_ptr[2];
	uint8_t *accuracy = (uint8_t *)callback_ref;
	char *event_text;

	if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
		event_text = "[META EVENT]";
	} else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
		event_text = "[META EVENT WAKE UP]";
	} else {
		return;
	}

	switch (meta_event_type) {
	case BHY2_META_EVENT_FLUSH_COMPLETE:
		printf("%s Flush complete for sensor id %u\r\n", event_text, byte1);
		break;
	case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
		printf("%s Sample rate changed for sensor id %u, now %u\r\n", event_text, byte1,
		       byte2);
		if (byte1 == KLIO_SENSOR_ID) {
			klio_set_imu_odr(byte2);
			klio_imu_state_update(IMU_STATE_ODR, byte2);
		}
		break;
	case BHY2_META_EVENT_POWER_MODE_CHANGED:
		printf("%s Power mode changed for sensor id %u, now %u\r\n", event_text, byte1,
		       byte2);
		if (byte1 == KLIO_SENSOR_ID) {

			klio_set_imu_started(byte2 == 7 ? 1 : 0);
			klio_imu_state_update(IMU_STATE_POWER_MODE, byte2 == 7 ? 1 : 0);
			if (klio_get_imu_started()) {
				if (klio_get_learning_started()) {
					klio_imu_state_update(IMU_STATE_LEARNING, 1);
				} else if (klio_get_recognition_started()) {
					klio_imu_state_update(IMU_STATE_RECOGNITION, 1);
				}
			} else {
				klio_imu_state_update(IMU_STATE_LEARNING, 0);
				klio_imu_state_update(IMU_STATE_RECOGNITION, 0);
			}
		}
		break;
	case BHY2_META_EVENT_ALGORITHM_EVENTS:
		printf("%s Algorithm event\r\n", event_text);
		break;
	case BHY2_META_EVENT_SENSOR_STATUS:
		printf("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
		if (accuracy) {
			if (byte1 == ACC_SENSOR_ID) {
				accuracy[0] = byte2;
			} else if (byte1 == GYRO_SENSOR_ID) {
				accuracy[1] = byte2;
			} else if (byte1 == EULER_SENSOR_ID) {
				accuracy[2] = byte2;
			}
		}
		break;
	case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
		printf("%s BSX event (do steps main)\r\n", event_text);
		break;
	case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
		printf("%s BSX event (do steps calib)\r\n", event_text);
		break;
	case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
		printf("%s BSX event (get output signal)\r\n", event_text);
		break;
	case BHY2_META_EVENT_SENSOR_ERROR:
		printf("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
		break;
	case BHY2_META_EVENT_FIFO_OVERFLOW:
		printf("%s FIFO overflow\r\n", event_text);
		break;
	case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
		printf("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
		break;
	case BHY2_META_EVENT_FIFO_WATERMARK:
		printf("%s FIFO watermark reached\r\n", event_text);
		break;
	case BHY2_META_EVENT_INITIALIZED:
		printf("%s Firmware initialized. Firmware version %u\r\n", event_text,
		       ((uint16_t)byte2 << 8) | byte1);
		break;
	case BHY2_META_TRANSFER_CAUSE:
		printf("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
		break;
	case BHY2_META_EVENT_SENSOR_FRAMEWORK:
		printf("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
		break;
	case BHY2_META_EVENT_RESET:
		printf("%s Reset event\r\n", event_text);
		break;
	case BHY2_META_EVENT_SPACER:
		break;
	default:
		printf("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
		break;
	}
}

int8_t bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;
	uint8_t tx[length + 1];
	uint8_t rx[length + 1];

	tx[0] = reg_addr;
	memcpy(tx + 1, reg_data, length);

	struct spi_buf tx_buf = {.buf = &tx, .len = length + 1};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf = {.buf = &rx, .len = length + 1};
	struct spi_buf_set rx_bufs = {.buffers = &rx_buf, .count = 1};

	spi_transceive(bmi_spi, &bmi_spi_cfg, &tx_bufs, &rx_bufs);

	memcpy(reg_data, rx + 1, length);

	return BHY2_OK;
}

int8_t bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	(void)intf_ptr;
	uint8_t tx[length + 1];

	tx[0] = reg_addr;
	memcpy(tx + 1, reg_data, length);

	struct spi_buf tx_buf = {.buf = &tx, .len = length + 1};
	struct spi_buf_set tx_bufs = {.buffers = &tx_buf, .count = 1};

	spi_write(bmi_spi, &bmi_spi_cfg, &tx_bufs);

	return BHY2_OK;
}

void bhy2_delay_us(uint32_t period, void *intf_ptr)
{
	(void)intf_ptr;

	k_sleep(K_USEC(period));
}

void print_api_error(int8_t rslt, struct bhy2_dev *dev, char *file, int line)
{
	(void)dev;
	if (rslt != BHY2_OK) {
		LOG_ERR("API Error %i file: %s @line %i", rslt, file, line);
		while (true) {
		}
	}
}

char *get_klio_error(bhy2_klio_driver_error_state_t error)
{
	char *ret = "";

	switch (error) {
	case KLIO_DRIVER_ERROR_NONE:
		break;
	case KLIO_DRIVER_ERROR_INVALID_PARAMETER:
		ret = "[Klio error] Invalid parameter";
		break;
	case KLIO_DRIVER_ERROR_PARAMETER_OUT_OF_RANGE:
		ret = "[Klio error] Parameter out of range";
		break;
	case KLIO_DRIVER_ERROR_INVALID_PATTERN_OPERATION:
		ret = "[Klio error] Invalid pattern operation";
		break;
	case KLIO_DRIVER_ERROR_NOT_IMPLEMENTED:
		ret = "[Klio error] Not implemented";
		break;
	case KLIO_DRIVER_ERROR_BUFSIZE:
		ret = "[Klio error] Buffer size";
		break;
	case KLIO_DRIVER_ERROR_INTERNAL:
		ret = "[Klio error] Internal";
		break;
	case KLIO_DRIVER_ERROR_UNDEFINED:
		ret = "[Klio error] Undefined";
		break;
	case KLIO_DRIVER_ERROR_OPERATION_PENDING:
		ret = "[Klio error] Operation pending";
		break;
	default:
		ret = "[Klio error] Unknown error code";
	}

	return ret;
}

char *get_sensor_name(uint8_t sensor_id)
{
	char *ret;

	switch (sensor_id) {
	case BHY2_SENSOR_ID_ACC_PASS:
		ret = "Accelerometer passthrough";
		break;
	case BHY2_SENSOR_ID_ACC_RAW:
		ret = "Accelerometer uncalibrated";
		break;
	case BHY2_SENSOR_ID_ACC:
		ret = "Accelerometer corrected";
		break;
	case BHY2_SENSOR_ID_ACC_BIAS:
		ret = "Accelerometer offset";
		break;
	case BHY2_SENSOR_ID_ACC_WU:
		ret = "Accelerometer corrected wake up";
		break;
	case BHY2_SENSOR_ID_ACC_RAW_WU:
		ret = "Accelerometer uncalibrated wake up";
		break;
	case BHY2_SENSOR_ID_GYRO_PASS:
		ret = "Gyroscope passthrough";
		break;
	case BHY2_SENSOR_ID_GYRO_RAW:
		ret = "Gyroscope uncalibrated";
		break;
	case BHY2_SENSOR_ID_GYRO:
		ret = "Gyroscope corrected";
		break;
	case BHY2_SENSOR_ID_GYRO_BIAS:
		ret = "Gyroscope offset";
		break;
	case BHY2_SENSOR_ID_GYRO_WU:
		ret = "Gyroscope wake up";
		break;
	case BHY2_SENSOR_ID_GYRO_RAW_WU:
		ret = "Gyroscope uncalibrated wake up";
		break;
	case BHY2_SENSOR_ID_MAG_PASS:
		ret = "Magnetometer passthrough";
		break;
	case BHY2_SENSOR_ID_MAG_RAW:
		ret = "Magnetometer uncalibrated";
		break;
	case BHY2_SENSOR_ID_MAG:
		ret = "Magnetometer corrected";
		break;
	case BHY2_SENSOR_ID_MAG_BIAS:
		ret = "Magnetometer offset";
		break;
	case BHY2_SENSOR_ID_MAG_WU:
		ret = "Magnetometer wake up";
		break;
	case BHY2_SENSOR_ID_MAG_RAW_WU:
		ret = "Magnetometer uncalibrated wake up";
		break;
	case BHY2_SENSOR_ID_GRA:
		ret = "Gravity vector";
		break;
	case BHY2_SENSOR_ID_GRA_WU:
		ret = "Gravity vector wake up";
		break;
	case BHY2_SENSOR_ID_LACC:
		ret = "Linear acceleration";
		break;
	case BHY2_SENSOR_ID_LACC_WU:
		ret = "Linear acceleration wake up";
		break;
	case BHY2_SENSOR_ID_RV:
		ret = "Rotation vector";
		break;
	case BHY2_SENSOR_ID_RV_WU:
		ret = "Rotation vector wake up";
		break;
	case BHY2_SENSOR_ID_GAMERV:
		ret = "Game rotation vector";
		break;
	case BHY2_SENSOR_ID_GAMERV_WU:
		ret = "Game rotation vector wake up";
		break;
	case BHY2_SENSOR_ID_GEORV:
		ret = "Geo-magnetic rotation vector";
		break;
	case BHY2_SENSOR_ID_GEORV_WU:
		ret = "Geo-magnetic rotation vector wake up";
		break;
	case BHY2_SENSOR_ID_ORI:
		ret = "Orientation";
		break;
	case BHY2_SENSOR_ID_ORI_WU:
		ret = "Orientation wake up";
		break;
	case BHY2_SENSOR_ID_ACC_BIAS_WU:
		ret = "Accelerometer offset wake up";
		break;
	case BHY2_SENSOR_ID_GYRO_BIAS_WU:
		ret = "Gyroscope offset wake up";
		break;
	case BHY2_SENSOR_ID_MAG_BIAS_WU:
		ret = "Magnetometer offset wake up";
		break;
	case BHY2_SENSOR_ID_TEMP:
		ret = "Temperature";
		break;
	case BHY2_SENSOR_ID_BARO:
		ret = "Barometer";
		break;
	case BHY2_SENSOR_ID_HUM:
		ret = "Humidity";
		break;
	case BHY2_SENSOR_ID_GAS:
		ret = "Gas";
		break;
	case BHY2_SENSOR_ID_TEMP_WU:
		ret = "Temperature wake up";
		break;
	case BHY2_SENSOR_ID_BARO_WU:
		ret = "Barometer wake up";
		break;
	case BHY2_SENSOR_ID_HUM_WU:
		ret = "Humidity wake up";
		break;
	case BHY2_SENSOR_ID_GAS_WU:
		ret = "Gas wake up";
		break;
	case BHY2_SENSOR_ID_KLIO:
		ret = "Klio";
		break;
	case BHY2_SENSOR_ID_KLIO_LOG:
		ret = "Klio log";
		break;
	case BHY2_SENSOR_ID_SI_ACCEL:
		ret = "SI Accel";
		break;
	case BHY2_SENSOR_ID_SI_GYROS:
		ret = "SI Gyro";
		break;
	case BHY2_SENSOR_ID_LIGHT:
		ret = "Light";
		break;
	case BHY2_SENSOR_ID_LIGHT_WU:
		ret = "Light wake up";
		break;
	case BHY2_SENSOR_ID_PROX:
		ret = "Proximity";
		break;
	case BHY2_SENSOR_ID_PROX_WU:
		ret = "Proximity wake up";
		break;
	case BHY2_SENSOR_ID_STC:
		ret = "Step counter";
		break;
	case BHY2_SENSOR_ID_STC_WU:
		ret = "Step counter wake up";
		break;
	case BHY2_SENSOR_ID_STC_LP:
		ret = "Low Power Step counter";
		break;
	case BHY2_SENSOR_ID_STC_LP_WU:
		ret = "Low Power Step counter wake up";
		break;
	case BHY2_SENSOR_ID_SIG:
		ret = "Significant motion";
		break;
	case BHY2_SENSOR_ID_STD:
		ret = "Step detector";
		break;
	case BHY2_SENSOR_ID_STD_WU:
		ret = "Step detector wake up";
		break;
	case BHY2_SENSOR_ID_TILT_DETECTOR:
		ret = "Tilt detector";
		break;
	case BHY2_SENSOR_ID_WAKE_GESTURE:
		ret = "Wake gesture";
		break;
	case BHY2_SENSOR_ID_GLANCE_GESTURE:
		ret = "Glance gesture";
		break;
	case BHY2_SENSOR_ID_PICKUP_GESTURE:
		ret = "Pickup gesture";
		break;
	case BHY2_SENSOR_ID_SIG_LP:
		ret = "Low Power Significant motion";
		break;
	case BHY2_SENSOR_ID_SIG_LP_WU:
		ret = "Low Power Significant motion wake up";
		break;
	case BHY2_SENSOR_ID_STD_LP:
		ret = "Low Power Step detector";
		break;
	case BHY2_SENSOR_ID_STD_LP_WU:
		ret = "Low Power Step detector wake up";
		break;
	case BHY2_SENSOR_ID_AR:
		ret = "Activity recognition";
		break;
	case BHY2_SENSOR_ID_EXCAMERA:
		ret = "External camera trigger";
		break;
	case BHY2_SENSOR_ID_GPS:
		ret = "GPS";
		break;
	case BHY2_SENSOR_ID_WRIST_TILT_GESTURE:
		ret = "Wrist tilt gesture";
		break;
	case BHY2_SENSOR_ID_DEVICE_ORI:
		ret = "Device orientation";
		break;
	case BHY2_SENSOR_ID_DEVICE_ORI_WU:
		ret = "Device orientation wake up";
		break;
	case BHY2_SENSOR_ID_STATIONARY_DET:
		ret = "Stationary detect";
		break;
	case BHY2_SENSOR_ID_ANY_MOTION_LP:
		ret = "Low Power Any motion";
		break;
	case BHY2_SENSOR_ID_ANY_MOTION_LP_WU:
		ret = "Low Power Any motion wake up";
		break;
	case BHY2_SENSOR_ID_MOTION_DET:
		ret = "Motion detect";
		break;
	default:
		if ((sensor_id >= BHY2_SENSOR_ID_CUSTOM_START) &&
		    (sensor_id <= BHY2_SENSOR_ID_CUSTOM_END)) {
			ret = "Custom sensor ID ";
		} else {
			ret = "Undefined sensor ID ";
		}
	}

	return ret;
}

char *get_sensor_error_text(uint8_t sensor_error)
{
	char *ret;

	switch (sensor_error) {
	case 0x00:
		break;
	case 0x10:
		ret = "[Sensor error] Bootloader reports: Firmware Expected Version "
		      "Mismatch";
		break;
	case 0x11:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad "
		      "Header CRC";
		break;
	case 0x12:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: SHA Hash "
		      "Mismatch";
		break;
	case 0x13:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad "
		      "Image CRC";
		break;
	case 0x14:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: ECDSA "
		      "Signature "
		      "Verification Failed";
		break;
	case 0x15:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Bad "
		      "Public Key "
		      "CRC";
		break;
	case 0x16:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: Signed "
		      "Firmware "
		      "Required";
		break;
	case 0x17:
		ret = "[Sensor error] Bootloader reports: Firmware Upload Failed: FW "
		      "Header "
		      "Missing";
		break;
	case 0x19:
		ret = "[Sensor error] Bootloader reports: Unexpected Watchdog Reset";
		break;
	case 0x1A:
		ret = "[Sensor error] ROM Version Mismatch";
		break;
	case 0x1B:
		ret = "[Sensor error] Bootloader reports: Fatal Firmware Error";
		break;
	case 0x1C:
		ret = "[Sensor error] Chained Firmware Error: Next Payload Not Found";
		break;
	case 0x1D:
		ret = "[Sensor error] Chained Firmware Error: Payload Not Valid";
		break;
	case 0x1E:
		ret = "[Sensor error] Chained Firmware Error: Payload Entries Invalid";
		break;
	case 0x1F:
		ret = "[Sensor error] Bootloader reports: Bootloader Error: OTP CRC "
		      "Invalid";
		break;
	case 0x20:
		ret = "[Sensor error] Firmware Init Failed";
		break;
	case 0x21:
		ret = "[Sensor error] Sensor Init Failed: Unexpected Device ID";
		break;
	case 0x22:
		ret = "[Sensor error] Sensor Init Failed: No Response from Device";
		break;
	case 0x23:
		ret = "[Sensor error] Sensor Init Failed: Unknown";
		break;
	case 0x24:
		ret = "[Sensor error] Sensor Error: No Valid Data";
		break;
	case 0x25:
		ret = "[Sensor error] Slow Sample Rate";
		break;
	case 0x26:
		ret = "[Sensor error] Data Overflow (saturated sensor data)";
		break;
	case 0x27:
		ret = "[Sensor error] Stack Overflow";
		break;
	case 0x28:
		ret = "[Sensor error] Insufficient Free RAM";
		break;
	case 0x29:
		ret = "[Sensor error] Sensor Init Failed: Driver Parsing Error";
		break;
	case 0x2A:
		ret = "[Sensor error] Too Many RAM Banks Required";
		break;
	case 0x2B:
		ret = "[Sensor error] Invalid Event Specified";
		break;
	case 0x2C:
		ret = "[Sensor error] More than 32 On Change";
		break;
	case 0x2D:
		ret = "[Sensor error] Firmware Too Large";
		break;
	case 0x2F:
		ret = "[Sensor error] Invalid RAM Banks";
		break;
	case 0x30:
		ret = "[Sensor error] Math Error";
		break;
	case 0x40:
		ret = "[Sensor error] Memory Error";
		break;
	case 0x41:
		ret = "[Sensor error] SWI3 Error";
		break;
	case 0x42:
		ret = "[Sensor error] SWI4 Error";
		break;
	case 0x43:
		ret = "[Sensor error] Illegal Instruction Error";
		break;
	case 0x44:
		ret = "[Sensor error] Bootloader reports: Unhandled Interrupt Error / "
		      "Exception / "
		      "Postmortem Available";
		break;
	case 0x45:
		ret = "[Sensor error] Invalid Memory Access";
		break;
	case 0x50:
		ret = "[Sensor error] Algorithm Error: BSX Init";
		break;
	case 0x51:
		ret = "[Sensor error] Algorithm Error: BSX Do Step";
		break;
	case 0x52:
		ret = "[Sensor error] Algorithm Error: Update Sub";
		break;
	case 0x53:
		ret = "[Sensor error] Algorithm Error: Get Sub";
		break;
	case 0x54:
		ret = "[Sensor error] Algorithm Error: Get Phys";
		break;
	case 0x55:
		ret = "[Sensor error] Algorithm Error: Unsupported Phys Rate";
		break;
	case 0x56:
		ret = "[Sensor error] Algorithm Error: Cannot find BSX Driver";
		break;
	case 0x60:
		ret = "[Sensor error] Sensor Self-Test Failure";
		break;
	case 0x61:
		ret = "[Sensor error] Sensor Self-Test X Axis Failure";
		break;
	case 0x62:
		ret = "[Sensor error] Sensor Self-Test Y Axis Failure";
		break;
	case 0x64:
		ret = "[Sensor error] Sensor Self-Test Z Axis Failure";
		break;
	case 0x65:
		ret = "[Sensor error] FOC Failure";
		break;
	case 0x66:
		ret = "[Sensor error] Sensor Busy";
		break;
	case 0x6F:
		ret = "[Sensor error] Self-Test or FOC Test Unsupported";
		break;
	case 0x72:
		ret = "[Sensor error] No Host Interrupt Set";
		break;
	case 0x73:
		ret = "[Sensor error] Event ID Passed to Host Interface Has No Known Size";
		break;
	case 0x75:
		ret = "[Sensor error] Host Download Channel Underflow (Host Read Too Fast)";
		break;
	case 0x76:
		ret = "[Sensor error] Host Upload Channel Overflow (Host Wrote Too Fast)";
		break;
	case 0x77:
		ret = "[Sensor error] Host Download Channel Empty";
		break;
	case 0x78:
		ret = "[Sensor error] DMA Error";
		break;
	case 0x79:
		ret = "[Sensor error] Corrupted Input Block Chain";
		break;
	case 0x7A:
		ret = "[Sensor error] Corrupted Output Block Chain";
		break;
	case 0x7B:
		ret = "[Sensor error] Buffer Block Manager Error";
		break;
	case 0x7C:
		ret = "[Sensor error] Input Channel Not Word Aligned";
		break;
	case 0x7D:
		ret = "[Sensor error] Too Many Flush Events";
		break;
	case 0x7E:
		ret = "[Sensor error] Unknown Host Channel Error";
		break;
	case 0x81:
		ret = "[Sensor error] Decimation Too Large";
		break;
	case 0x90:
		ret = "[Sensor error] Master SPI/I2C Queue Overflow";
		break;
	case 0x91:
		ret = "[Sensor error] SPI/I2C Callback Error";
		break;
	case 0xA0:
		ret = "[Sensor error] Timer Scheduling Error";
		break;
	case 0xB0:
		ret = "[Sensor error] Invalid GPIO for Host IRQ";
		break;
	case 0xB1:
		ret = "[Sensor error] Error Sending Initialized Meta Events";
		break;
	case 0xC0:
		ret = "[Sensor error] Bootloader reports: Command Error";
		break;
	case 0xC1:
		ret = "[Sensor error] Bootloader reports: Command Too Long";
		break;
	case 0xC2:
		ret = "[Sensor error] Bootloader reports: Command Buffer Overflow";
		break;
	case 0xD0:
		ret = "[Sensor error] User Mode Error: Sys Call Invalid";
		break;
	case 0xD1:
		ret = "[Sensor error] User Mode Error: Trap Invalid";
		break;
	case 0xE1:
		ret = "[Sensor error] Firmware Upload Failed: Firmware header corrupt";
		break;
	case 0xE2:
		ret = "[Sensor error] Sensor Data Injection: Invalid input stream";
		break;
	default:
		ret = "[Sensor error] Unknown error code";
	}

	return ret;
}
