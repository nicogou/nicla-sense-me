#include <zephyr/sys/util.h>
#include <zephyr/drivers/charger.h>

// Register Map
// https://www.ti.com/lit/ds/symlink/bq25120a.pdf
#define BQ25120A_STATUS           0x00
#define BQ25120A_FAULTS           0x01
#define BQ25120A_TS_CONTROL       0x02
#define BQ25120A_FAST_CHG         0x03
#define BQ25120A_TERMINATION_CURR 0x04
#define BQ25120A_BATTERY_CTRL     0x05 // Battery Voltage Control Register
#define BQ25120A_SYS_VOUT_CTRL    0x06
#define BQ25120A_LDO_CTRL         0x07
#define BQ25120A_PUSH_BUTT_CTRL   0x08
#define BQ25120A_ILIM_UVLO_CTRL   0x09
#define BQ25120A_BATT_MON         0x0A
#define BQ25120A_VIN_DPM          0x0B

#define BQ25120A_LDO_ENABLE_MASK BIT(7)
#define BQ25120A_LDO_ENABLE      0x01
#define BQ25120A_LDO_DISABLE     0x00
#define BQ25120A_LDO_VALUE_MASK  GENMASK(6, 2)

#define BQ25120A_STATUS_VALUE_NOT_CHARGING 	0x00
#define BQ25120A_STATUS_VALUE_CHARGING 		0x01
#define BQ25120A_STATUS_VALUE_FULL			0x02
#define BQ25120A_STATUS_VALUE_FAULT 		0x03

enum bq25120a_charger_property {
	BQ25120A_CHARGER_PROP_LDO_VALUE = CHARGER_PROP_CUSTOM_BEGIN,
};

union bq25120a_propval {
	union charger_propval charger_propval;
	uint32_t ldo_value;
};
