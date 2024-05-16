#include "app.h"

/* Forward declaration of state table */
static const struct smf_state app_states[];

/* User defined object */
struct s_object s_obj;

/* State IDLE */
static void idle_entry(void *o)
{
	/* Do something */
}
static void idle_run(void *o)
{
	/* Do something */
}
static void idle_exit(void *o)
{
	/* Do something */
}

/* State RECORDING */
static void recording_entry(void *o)
{
	/* Do something */
}
static void recording_run(void *o)
{
	/* Do something */
}
static void recording_exit(void *o)
{
	/* Do something */
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
	int ret;
	/* Run the state machine */
	while (1) {
		/* State machine terminates if a non-zero value is returned */
		ret = smf_run_state(SMF_CTX(&s_obj));
		if (ret) {
			/* handle return code and terminate state machine */
			break;
		}
		k_msleep(1000);
	}

	return -1;
}
