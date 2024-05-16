#include "nicla_zbus.h"

LOG_MODULE_DECLARE(nicla_zbus, CONFIG_APP_LOG_LEVEL);

/* Define Instructions Channel */
/* This channel has a single observer as a message subscriber: the main thread*/
ZBUS_CHAN_DEFINE(instructions_chan, /* Name */
		 instruction_msg_t, /* Message type */

		 NULL,                             /* Validator */
		 NULL,                             /* User data */
		 ZBUS_OBSERVERS(app_zbus_msg_sub), /* observers */
		 ZBUS_MSG_INIT(.source = INSTRUCTION_SOURCE_COUNT,
			       .type = NICLA_ZBUS_UNKNOWN_TYPE) /* Initial value */
);

ZBUS_MSG_SUBSCRIBER_DEFINE(app_zbus_msg_sub);
