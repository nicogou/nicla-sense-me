/*
 * Copyright 2024 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * bq25120a Datasheet: https://www.ti.com/lit/gpn/bq25120a
 */

#define DT_DRV_COMPAT ti_bq25120a

#include <zephyr/device.h>
#include <zephyr/drivers/charger.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <app/drivers/bq25120a.h>

LOG_MODULE_REGISTER(bq25120a, CONFIG_CHARGER_LOG_LEVEL);

struct bq25120a_config {
	struct i2c_dt_spec i2c;
};

static int bq25120a_set_ldo_value(const struct device *dev, uint32_t ldo_voltage_value)
{
	const struct bq25120a_config *cfg = dev->config;
	uint8_t val = 0;
	int ret;

	if ((ldo_voltage_value > 0 && ldo_voltage_value <= 8) || ldo_voltage_value > 39) {
		LOG_ERR("LDO voltage value out of bounds: voltage must be between 0.9V and 3.9V.");
		return -EINVAL;
	}

	if (ldo_voltage_value > 0) {
		// Enable LDO.
		val |= FIELD_PREP(BQ25120A_LDO_ENABLE_MASK, BQ25120A_LDO_ENABLE);
		val |= FIELD_PREP(BQ25120A_LDO_VALUE_MASK, (uint8_t)(ldo_voltage_value - 8));
	} else {
		// Disable LDO.
		val |= FIELD_PREP(BQ25120A_LDO_ENABLE_MASK, BQ25120A_LDO_DISABLE);
	}

	ret = i2c_reg_update_byte_dt(&cfg->i2c, BQ25120A_LDO_CTRL,
				     BQ25120A_LDO_ENABLE_MASK | BQ25120A_LDO_VALUE_MASK, val);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int bq25120a_get_prop(const struct device *dev, charger_prop_t prop,
			     union charger_propval *val)
{
	switch (prop) {
	default:
		return -ENOTSUP;
	}
}

static int bq25120a_set_prop(const struct device *dev, charger_prop_t prop,
			     const union charger_propval *val)
{
	switch (prop) {
	case BQ25120A_CHARGER_PROP_LDO_VALUE:
		return bq25120a_set_ldo_value(dev, val->const_charge_voltage_uv);
	default:
		return -ENOTSUP;
	}
}

static int bq25120a_charge_enable(const struct device *dev, const bool enable)
{
	return -ENOTSUP;
}

static const struct charger_driver_api bq25120a_api = {
	.get_property = bq25120a_get_prop,
	.set_property = bq25120a_set_prop,
	.charge_enable = bq25120a_charge_enable,
};

static int bq25120a_init(const struct device *dev)
{
	const struct bq25120a_config *cfg = dev->config;
	uint8_t val;
	int ret;

	ret = i2c_reg_read_byte_dt(&cfg->i2c, 0x00, &val);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_BQ25120A_ENABLE_LDO_ON_STARTUP
	if (CONFIG_BQ25120A_LDO_STARTUP_VALUE != 0) {
		const union bq25120a_propval ldo = {.ldo_value = CONFIG_BQ25120A_LDO_STARTUP_VALUE};
		bq25120a_set_prop(dev, BQ25120A_CHARGER_PROP_LDO_VALUE, &ldo);
	}
#endif // CONFIG_BQ25120A_ENABLE_LDO_ON_STARTUP

	return 0;
}

#define CHARGER_BQ25120A_INIT(inst)                                                                \
	static const struct bq25120a_config bq25120a_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, bq25120a_init, NULL, NULL, &bq25120a_config_##inst,            \
			      POST_KERNEL, CONFIG_CHARGER_INIT_PRIORITY, &bq25120a_api);

DT_INST_FOREACH_STATUS_OKAY(CHARGER_BQ25120A_INIT)
