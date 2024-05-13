#define DT_DRV_COMPAT issi_is31fl3194

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(is31fl3194, CONFIG_LED_LOG_LEVEL);

// Regsiter Map
// http://www.issi.com/WW/pdf/IS31FL3194.pdf
#define IS31FL3194_PRODUCT_ID_VALUE     0xCE
#define IS31FL3194_BRIGHTNESS_MAX_VALUE 0x08

#define IS31FL3194_PRODUCT_ID    0x00 // should return IS31FL3194_PRODUCT_ID_VALUE: 0xCE
#define IS31FL3194_OP_CONFIG     0x01
#define IS31FL3194_OUT_CONFIG    0x02
#define IS31FL3194_CURRENT_BAND  0x03
#define IS31FL3194_HOLD_FUNCTION 0x04

#define IS31FL3194_P1_STATE 0x0D
#define IS31FL3194_P2_STATE 0x0E
#define IS31FL3194_P3_STATE 0x0F

// Current Mode
#define IS31FL3194_OUT1 0x10
#define IS31FL3194_OUT2 0x21
#define IS31FL3194_OUT3 0x32

// Pattern mode
//  Colors 1, 2 and 3 of pattern 1
#define IS31FL3194_COL1_PATT1_R 0x10
#define IS31FL3194_COL1_PATT1_G 0x11
#define IS31FL3194_COL1_PATT1_B 0x12
#define IS31FL3194_COL2_PATT1_R 0x13
#define IS31FL3194_COL2_PATT1_G 0x14
#define IS31FL3194_COL2_PATT1_B 0x15
#define IS31FL3194_COL3_PATT1_R 0x16
#define IS31FL3194_COL3_PATT1_G 0x17
#define IS31FL3194_COL3_PATT1_B 0x18

// Colors 1, 2 and 3 of pattern 2
#define IS31FL3194_COL1_PATT2_R 0x20
#define IS31FL3194_COL1_PATT2_G 0x21
#define IS31FL3194_COL1_PATT2_B 0x22
#define IS31FL3194_COL2_PATT2_R 0x23
#define IS31FL3194_COL2_PATT2_G 0x24
#define IS31FL3194_COL2_PATT2_B 0x25
#define IS31FL3194_COL3_PATT2_R 0x26
#define IS31FL3194_COL3_PATT2_G 0x27
#define IS31FL3194_COL3_PATT2_B 0x28

// Colors 1, 2 and 3 of pattern 3
#define IS31FL3194_COL1_PATT3_R 0x30
#define IS31FL3194_COL1_PATT3_G 0x31
#define IS31FL3194_COL1_PATT3_B 0x32
#define IS31FL3194_COL2_PATT3_R 0x33
#define IS31FL3194_COL2_PATT3_G 0x34
#define IS31FL3194_COL2_PATT3_B 0x35
#define IS31FL3194_COL3_PATT3_R 0x36
#define IS31FL3194_COL3_PATT3_G 0x37
#define IS31FL3194_COL3_PATT3_B 0x38

#define IS31FL3194_P1_TS_T1_Time_SET 0x19
#define IS31FL3194_P1_T2_T3_Time_SET 0x1A
#define IS31FL3194_P1_TP_T4_Time_SET 0x1B
#define IS31FL3194_P2_TS_T1_Time_SET 0x29
#define IS31FL3194_P2_T2_T3_Time_SET 0x2A
#define IS31FL3194_P2_TP_T4_Time_SET 0x2B
#define IS31FL3194_P3_TS_T1_Time_SET 0x39
#define IS31FL3194_P3_T2_T3_Time_SET 0x3A
#define IS31FL3194_P3_TP_T4_Time_SET 0x3B

#define IS31FL3194_P1_COLOR_EN 0x1C
#define IS31FL3194_P2_COLOR_EN 0x2C
#define IS31FL3194_P3_COLOR_EN 0x3C

#define IS31FL3194_P1_COLOR_CYC_TIME 0x1D
#define IS31FL3194_P2_COLOR_CYC_TIME 0x2D
#define IS31FL3194_P3_COLOR_CYC_TIME 0x3D

#define IS31FL3194_P1_NXT 0x1E
#define IS31FL3194_P2_NXT 0x2E
#define IS31FL3194_P3_NXT 0x3E

#define IS31FL3194_P1_LOOP_TIMES 0x1F
#define IS31FL3194_P2_LOOP_TIMES 0x2F
#define IS31FL3194_P3_LOOP_TIMES 0x3F

#define IS31FL3194_COLOR_UPDATE 0x40

#define IS31FL3194_P1_UPDATE 0x41
#define IS31FL3194_P2_UPDATE 0x42
#define IS31FL3194_P3_UPDATE 0x43

#define IS31FL3194_RESET 0x4F

#define IS31FL3194_ADDRESS 0x53

#define Mode 1

// define times
#define t_0_03s 0x00
#define t_0_13s 0x01
#define t_0_26s 0x02
#define t_0_38s 0x03
#define t_0_51s 0x04
#define t_0_77s 0x05
#define t_1_04s 0x06
#define t_1_60s 0x07
#define t_2_10s 0x08
#define t_2_60s 0x09
#define t_3_10s 0x0A
#define t_4_20s 0x0B
#define t_5_20s 0x0C
#define t_6_20s 0x0D
#define t_7_30s 0x0E
#define t_8_30s 0x0F

// define pattern times
#define TS t_2_10s // Start time
#define T1 t_2_10s // Rise time
#define T2 t_1_04s // Hold time
#define T3 t_2_10s // Fall time
#define T4 t_2_10s // Off time
#define TP t_2_10s // Time between pulses

// define cycle times
#define endless 0x00
#define once    0x15
#define twice   0x2A
#define thrice  0x3F

// light intensity (fraction of current max)
#define Imax_frac 0x80 // Imax_frac/256 * Imax = current

struct is31fl3194_cfg {
	struct i2c_dt_spec i2c;
	uint8_t num_leds;
	const struct led_info *leds_info;
	uint8_t brightness;
};

static const struct led_info *is31fl3194_led_to_info(const struct is31fl3194_cfg *config,
						     uint32_t led)
{
	if (led < config->num_leds) {
		return &config->leds_info[led];
	}

	return NULL;
}

static int is31fl3194_led_get_info(const struct device *dev, uint32_t led,
				   const struct led_info **info)
{
	const struct is31fl3194_cfg *config = dev->config;
	const struct led_info *led_info = is31fl3194_led_to_info(config, led);

	if (!led_info) {
		return -EINVAL;
	}

	*info = led_info;

	return 0;
}

static int is31fl3194_write_buffer(const struct i2c_dt_spec *i2c, const uint8_t *buffer,
				   uint32_t num_bytes)
{
	int status;

	status = i2c_write_dt(i2c, buffer, num_bytes);
	if (status < 0) {
		LOG_ERR("Could not write buffer: %i", status);
		return status;
	}

	return 0;
}

static int is31fl3194_write_reg(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t val)
{
	uint8_t buffer[2] = {reg, val};

	return is31fl3194_write_buffer(i2c, buffer, sizeof(buffer));
}

static int is31fl3194_read_buffer(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buffer,
				  uint32_t num_bytes)
{
	int status;

	status = i2c_burst_read_dt(i2c, reg, buffer, num_bytes);
	if (status < 0) {
		LOG_ERR("Could not read reg %u: %i", reg, status);
		return status;
	}

	return 0;
}

static int is31fl3194_read_reg(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *val)
{
	return is31fl3194_read_buffer(i2c, reg, val, 1);
}

static int is31fl3194_init_registers(const struct i2c_dt_spec *i2c)
{
	int status;

	// Reset chip before starting
	status = is31fl3194_write_reg(i2c, IS31FL3194_RESET, 0xC5);
	if (status < 0) {
		return status;
	}

	uint8_t chip_id;
	status = is31fl3194_read_reg(i2c, IS31FL3194_PRODUCT_ID, &chip_id);
	if (status < 0) {
		return status;
	}

	if (chip_id != IS31FL3194_PRODUCT_ID_VALUE) {
		LOG_ERR("Wrong IS31FL3194 chip id read: %u", chip_id);
		return -EFAULT;
	}

	status = is31fl3194_write_reg(i2c, IS31FL3194_OP_CONFIG,
				      0x01); // normal operation in current mode
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(i2c, IS31FL3194_OUT_CONFIG, 0x07); // enable all three ouputs
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(i2c, IS31FL3194_CURRENT_BAND, 0x00); // 10 mA max current
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(i2c, IS31FL3194_HOLD_FUNCTION, 0x00); // hold function disable
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3194_init(const struct device *dev)
{
	int status;
	const struct is31fl3194_cfg *config = dev->config;

	LOG_DBG("Initializing @0x%x...", config->i2c.addr);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	status = is31fl3194_init_registers(&config->i2c);
	if (status < 0) {
		LOG_ERR("Failed to initialize IS31FL3194 registers (%u)", status);
	}

	return 0;
}

static int is31fl3194_led_set_color(const struct device *dev, uint32_t led, uint8_t num_colors,
				    const uint8_t *color)
{
	int status;
	const struct is31fl3194_cfg *config = dev->config;
	const struct led_info *led_info = is31fl3194_led_to_info(config, led);
	uint8_t scale_factor = IS31FL3194_BRIGHTNESS_MAX_VALUE - config->brightness;

	if (!led_info) {
		return -ENODEV;
	}

	if (led_info->num_colors != 3) {
		return -ENOTSUP;
	}
	if (num_colors != 3) {
		return -EINVAL;
	}

	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_OUT1,
				      color[2] >> scale_factor); // write channel 1
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_OUT2,
				      color[1] >> scale_factor); // write channel 2
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_OUT3,
				      color[0] >> scale_factor); // write channel 3
	if (status < 0) {
		return status;
	}

	// Pattern 1 color
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT1_R,
				      color[2] >> scale_factor); // color 1
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT1_G,
				      color[2] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT1_B,
				      color[2] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT1_R,
				      color[2] >> scale_factor); // color 2
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT1_G,
				      color[2] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT1_B,
				      color[2] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT1_R,
				      color[2] >> scale_factor); // color 3
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT1_G,
				      color[2] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT1_B,
				      color[2] >> scale_factor);

	// Pattern 2 color
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT2_R,
				      color[1] >> scale_factor); // color 1
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT2_G,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT2_B,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT2_R,
				      color[1] >> scale_factor); // color 2
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT2_G,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT2_B,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT2_R,
				      color[1] >> scale_factor); // color 3
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT2_G,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT2_B,
				      color[1] >> scale_factor);
	if (status < 0) {
		return status;
	}
	// Pattern 3 color
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT3_R,
				      color[0] >> scale_factor); // color 1
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT3_G,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL1_PATT3_B,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT3_R,
				      color[0] >> scale_factor); // color 2
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT3_G,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL2_PATT3_B,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT3_R,
				      color[0] >> scale_factor); // color 3
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT3_G,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COL3_PATT3_B,
				      color[0] >> scale_factor);
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3194_led_on(const struct device *dev, uint32_t led)
{
	int status;
	const struct is31fl3194_cfg *config = dev->config;
	const struct led_info *led_info = is31fl3194_led_to_info(config, led);

	if (!led_info) {
		return -ENODEV;
	}

	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_OP_CONFIG,
				      0x01); // normal operation in current mode
	if (status < 0) {
		return status;
	}

	status = is31fl3194_write_reg(
		&config->i2c, IS31FL3194_COLOR_UPDATE,
		0xC5); // write to color update register for changes to take effect.
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3194_led_off(const struct device *dev, uint32_t led)
{
	int status;
	const struct is31fl3194_cfg *config = dev->config;
	const struct led_info *led_info = is31fl3194_led_to_info(config, led);

	if (!led_info) {
		return -ENODEV;
	}

	uint8_t off[3] = {0, 0, 0};
	status = is31fl3194_led_set_color(dev, led, led_info->num_colors, off);
	if (status < 0) {
		return status;
	}

	status = is31fl3194_led_on(dev, led);
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3194_led_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	LOG_WRN("Not implemented.");
	return -ENOTSUP;
}

static int is31fl3194_led_blink(const struct device *dev, uint32_t led, uint32_t delay_on,
				uint32_t delay_off)
{
	int status;
	const struct is31fl3194_cfg *config = dev->config;
	const struct led_info *led_info = is31fl3194_led_to_info(config, led);

	if (!led_info) {
		return -ENODEV;
	}

	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_OP_CONFIG,
				      0x71); // select pattern mode.
	if (status < 0) {
		return status;
	}

	uint8_t t1_ts = t_0_03s << 4 | t_0_03s;
	uint8_t t3_t2 = t_0_03s << 4 | (delay_on & 0x0F);
	uint8_t tp_t4 = t_0_03s << 4 | (delay_off & 0x0F);
	// Pattern 1 timing
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_TS_T1_Time_SET,
				      t1_ts); // Rise time (T1) and start time (TS)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_T2_T3_Time_SET,
				      t3_t2); // Hold time (T2) and fall time (T3)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_TP_T4_Time_SET,
				      tp_t4); // Off time (T4) and time between pulses (TP)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_COLOR_CYC_TIME, once);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_NXT, 0x00);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_LOOP_TIMES, 0x00);
	if (status < 0) {
		return status;
	}

	// Pattern 2 timing
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_TS_T1_Time_SET,
				      t1_ts); // Rise time (T1) and start time (TS)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_T2_T3_Time_SET,
				      t3_t2); // Hold time (T2) and fall time (T3)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_TP_T4_Time_SET,
				      tp_t4); // Off time (T4) and time between pulses (TP)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_COLOR_CYC_TIME, once);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_NXT, 0x00);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_LOOP_TIMES, 0x00);
	if (status < 0) {
		return status;
	}

	// Pattern 3 timing
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_TS_T1_Time_SET,
				      t1_ts); // Rise time (T1) and start time (TS)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_T2_T3_Time_SET,
				      t3_t2); // Hold time (T2) and fall time (T3)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_TP_T4_Time_SET,
				      tp_t4); // Off time (T4) and time between pulses (TP)
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_COLOR_CYC_TIME, once);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_NXT, 0x00);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_LOOP_TIMES, 0x00);
	if (status < 0) {
		return status;
	}

	// Enable and update
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_COLOR_EN, 0x01);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_COLOR_EN, 0x01);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_COLOR_EN, 0x01);
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_COLOR_UPDATE,
				      0xC5); // update colors
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P1_UPDATE,
				      0xC5); // launch pattern 1
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P2_UPDATE,
				      0xC5); // launch pattern 1
	if (status < 0) {
		return status;
	}
	status = is31fl3194_write_reg(&config->i2c, IS31FL3194_P3_UPDATE,
				      0xC5); // launch pattern 1
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3194_led_write_channels(const struct device *dev, uint32_t start_channel,
					 uint32_t num_channels, const uint8_t *buf)
{
	LOG_ERR("Not supported.");
	return -ENOTSUP;
}

static const struct led_driver_api is31fl3194_led_api = {
	.set_brightness = is31fl3194_led_set_brightness,
	.on = is31fl3194_led_on,
	.off = is31fl3194_led_off,
	.set_color = is31fl3194_led_set_color,
	.get_info = is31fl3194_led_get_info,
	.write_channels = is31fl3194_led_write_channels,
	.blink = is31fl3194_led_blink,
};

#define COLOR_MAPPING(led_node_id)                                                                 \
	static const uint8_t color_mapping_##led_node_id[] = DT_PROP(led_node_id, color_mapping);

#define LED_INFO(led_node_id)                                                                      \
	{                                                                                          \
		.label = DT_PROP(led_node_id, label),                                              \
		.index = DT_PROP(led_node_id, index),                                              \
		.num_colors = DT_PROP_LEN(led_node_id, color_mapping),                             \
		.color_mapping = color_mapping_##led_node_id,                                      \
	},

#define IS31FL3194_INIT(id)                                                                        \
                                                                                                   \
	DT_INST_FOREACH_CHILD(id, COLOR_MAPPING)                                                   \
                                                                                                   \
	static const struct led_info is31fl3194_leds_##id[] = {                                    \
		DT_INST_FOREACH_CHILD(id, LED_INFO)};                                              \
                                                                                                   \
	static const struct is31fl3194_cfg is31fl3194_cfg_##id = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(id),                                                   \
		.num_leds = ARRAY_SIZE(is31fl3194_leds_##id),                                      \
		.leds_info = is31fl3194_leds_##id,                                                 \
		.brightness = DT_INST_PROP(id, brightness),                                        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(id, &is31fl3194_init, NULL, NULL, &is31fl3194_cfg_##id, POST_KERNEL, \
			      CONFIG_LED_INIT_PRIORITY, &is31fl3194_led_api);

DT_INST_FOREACH_STATUS_OKAY(IS31FL3194_INIT)
