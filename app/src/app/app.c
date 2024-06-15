#include "app.h"
#include <nicla_sd/nicla_sd.h>

LOG_MODULE_REGISTER(app, CONFIG_APP_LOG_LEVEL);

ZBUS_CHAN_DECLARE(instructions_chan);
ZBUS_OBS_DECLARE(app_zbus_msg_sub);

/* Forward declaration of state table */
static const struct smf_state app_states[];

/* User defined object */
struct s_object s_obj;

/* State IDLE */
static void idle_entry(void *o)
{
	LOG_INF("Idle state");
}
static void idle_run(void *o)
{
	struct s_object *s = (struct s_object *)o;
	switch (s->instruction.source) {
	case INSTRUCTION_SOURCE_BLE:
		if (s->instruction.type == BLE_CONNECTED) {
			LOG_INF("BLE connected in Idle state");
		} else if (s->instruction.type == BLE_DISCONNECTED) {
			LOG_INF("BLE disonnected in Idle state");
		}
		break;

	case INSTRUCTION_SOURCE_APP:
		if (s->instruction.type == RECORDING_START) {
			smf_set_state(SMF_CTX(&s_obj), &app_states[RECORDING]);
		}
		break;

	default:
		LOG_WRN("Unhandled instruction in Idle state : %u", s->instruction.source);
		break;
	}
}
static void idle_exit(void *o)
{
	LOG_INF("Leaving Idle state");
}

/* State RECORDING */
static void recording_entry(void *o)
{
	LOG_INF("Recording state");
	nicla_sd_init();
	int res = nicla_sd_create_session();
	if (res < 0){
		LOG_WRN("Unable to create session directory, returning to idle state.");
		instruction_msg_t msg = {.source = INSTRUCTION_SOURCE_APP, .type = RECORDING_STOP};
		zbus_chan_pub(&instructions_chan, &msg, K_MSEC(100));
	} else {
		LOG_INF("Creating session n°%i", res);
		imu_start(IMU_SAMPLE_RATE, 1000);
	}
}
static void recording_run(void *o)
{
	struct s_object *s = (struct s_object *)o;
	switch (s->instruction.source) {
	case INSTRUCTION_SOURCE_BLE:
		if (s->instruction.type == BLE_CONNECTED) {
			LOG_INF("BLE connected in Recording state");
		} else if (s->instruction.type == BLE_DISCONNECTED) {
			LOG_INF("BLE disonnected in Recording state");
		}
		break;

	case INSTRUCTION_SOURCE_APP:
		if (s->instruction.type == RECORDING_STOP) {
			/* Don't set state directly here. Instead stop the IMU and wait for meta events
			   to report that sample rate is down to 0 for both acc and gyro.
			*/
			imu_stop();
		} else if (s->instruction.type == RECORDING_START) {
			LOG_WRN("Recording already started");
		} else if (s->instruction.type == RECORDING_GO_TO_IDLE) {
			smf_set_state(SMF_CTX(&s_obj), &app_states[IDLE]);
		}
		break;

	default:
		LOG_WRN("Unhandled instruction in Recording state : %u", s->instruction.source);
		break;
	}
}
static void recording_exit(void *o)
{
	LOG_INF("Leaving Recording state");
	nicla_sd_end_current_session();
	nicla_sd_unmount();
}

/* Populate state table */
static const struct smf_state app_states[] = {
	[IDLE] = SMF_CREATE_STATE(idle_entry, idle_run, idle_exit),
	[RECORDING] = SMF_CREATE_STATE(recording_entry, recording_run, recording_exit),
};

int app_init()
{
	/* Set initial state */
	smf_set_initial(SMF_CTX(&s_obj), &app_states[IDLE]);

	return 0;
}

int app_run()
{
	int ret = 0;
	const struct zbus_channel *chan;
	instruction_msg_t inst;

	/* Run the state machine */
	while (!zbus_sub_wait_msg(&app_zbus_msg_sub, &chan, &inst, K_FOREVER)) {
		if (&instructions_chan != chan) {
			LOG_ERR("Wrong channel %p!", chan);
			continue;
		}

		s_obj.instruction = inst;
		/* State machine terminates if a non-zero value is returned */
		ret = smf_run_state(SMF_CTX(&s_obj));
		if (ret) {
			/* handle return code and terminate state machine */
			break;
		}
	}

	return ret;
}
