#include "nicla_service.h"

LOG_MODULE_REGISTER(nicla_service, LOG_LEVEL_DBG);

static struct nicla_service_cb nicla_service_cb;

static struct bt_gatt_indicate_params control_indicate_params = {0};

static void nicla_service_ccc_cfg_changed_control(const struct bt_gatt_attr *attr, uint16_t value)
{
	if (nicla_service_cb.control_send_enabled) {
		LOG_DBG("Indications has been turned %s (control)",
			value == BT_GATT_CCC_INDICATE ? "on" : "off");
		nicla_service_cb.control_send_enabled(value == BT_GATT_CCC_INDICATE
							      ? NICLA_SERVICE_SEND_STATUS_ENABLED
							      : NICLA_SERVICE_SEND_STATUS_DISABLED);
	}
}

static ssize_t control_on_receive(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Received data (Control), handle %d, conn %p", attr->handle, (void *)conn);

	if (nicla_service_cb.control_received) {
		nicla_service_cb.control_received(conn, buf, len);
	}
	return len;
}

static void control_on_sent(struct bt_conn *conn, struct bt_gatt_indicate_params *params,
			    uint8_t err)
{
	ARG_UNUSED(params);
	ARG_UNUSED(err);

	LOG_DBG("Data send (control), conn %p", (void *)conn);

	if (nicla_service_cb.control_sent) {
		nicla_service_cb.control_sent(conn);
	}
}

/* UART Service Declaration */
BT_GATT_SERVICE_DEFINE(nicla_service, BT_GATT_PRIMARY_SERVICE(BT_UUID_NICLA_SERVICE),

		       /* Control Characteristic */
		       BT_GATT_CHARACTERISTIC(BT_UUID_CONTROL,
					      BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE |
						      BT_GATT_CHRC_WRITE_WITHOUT_RESP,
#ifdef CONFIG_NICLA_SERVICE_AUTHEN
					      BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN,
#else
					      BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
#endif /* CONFIG_NICLA_SERVICE_AUTHEN */
					      NULL, control_on_receive, NULL),
		       BT_GATT_CCC(nicla_service_ccc_cfg_changed_control,
#ifdef CONFIG_NICLA_SERVICE_AUTHEN
				   BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
#else
				   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif /* CONFIG_NICLA_SERVICE_AUTHEN */
);

int nicla_service_init(struct nicla_service_cb *callbacks)
{
	if (callbacks) {
		nicla_service_cb.control_received = callbacks->control_received;
		nicla_service_cb.control_sent = callbacks->control_sent;
		nicla_service_cb.control_send_enabled = callbacks->control_send_enabled;
	}

	return 0;
}

int control_send(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
	const struct bt_gatt_attr *attr =
		bt_gatt_find_by_uuid(nicla_service.attrs, 1, BT_UUID_CONTROL);

	control_indicate_params.attr = attr;
	control_indicate_params.data = data;
	control_indicate_params.len = len;
	control_indicate_params.func = control_on_sent;

	if (!conn) {
		LOG_DBG("Indication sent to all connected peers (control)");
		return bt_gatt_indicate(NULL, &control_indicate_params);
	} else if (bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_INDICATE)) {
		return bt_gatt_indicate(conn, &control_indicate_params);
	} else {
		return -EINVAL;
	}
}
