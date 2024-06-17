#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */
#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT   "/" DISK_DRIVE_NAME ":"

#define MAX_PATH          128
#define SOME_FILE_NAME    "some.dat"
#define SOME_DIR_NAME     "some"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))

#define SESSION_DIR_NAME		"session_"
#define SESSION_ACC_FILE_NAME		"acc"
#define SESSION_GYRO_FILE_NAME		"gyro"
#define SESSION_FILE_EXTENSION		".csv"
#define SESSION_FILE_HEADER(file)	"timestamp,"file"_x,"file"_y,"file"_z\n"

int lsdir(const char *path);
bool create_some_entries(const char *base_path);
int nicla_sd_create_session();
int nicla_sd_end_current_session();
int nicla_sd_write(const char *file_name, int16_t *data, size_t length, uint64_t timestamp);
int nicla_sd_unmount();
void nicla_sd_init();
