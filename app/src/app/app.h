#pragma once

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

#include "common.h"
#include "nicla_zbus/nicla_zbus.h"

/* User defined object */
struct s_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Other state specific data add here */
	instruction_msg_t instruction;
};

int app_init();
int app_run();
