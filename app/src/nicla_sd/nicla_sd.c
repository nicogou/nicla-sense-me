#include "nicla_sd.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

LOG_MODULE_REGISTER(nicla_sd, CONFIG_APP_LOG_LEVEL);

static const char *disk_mount_pt = DISK_MOUNT_PT;

static int current_session_nb = 0;
struct fs_file_t current_session_acc_file;
struct fs_file_t current_session_gyro_file;

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static int get_session_nb(const char *path){
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_ERR("Error opening dir %s [%d]", path, res);
		return res;
	}

	int session_nb = 0;
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		/* In that case, create session_0 dir. */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			if (strncmp(entry.name, SESSION_DIR_NAME, 8) == 0) {
				errno = 0;
				uintmax_t tmp = strtoumax(&entry.name[8], NULL, 10); // Get directory number as uint
				if (tmp == UINTMAX_MAX && errno == ERANGE){
					LOG_ERR("Could not find directory number.");
				} else {
					if (tmp > session_nb) {
						session_nb = tmp;
					}
				}
			}
		} else {
		}
		count++;
	}

	/* Verify fs_closedir() */
	res = fs_closedir(&dirp);
	if (res != 0) {
		LOG_ERR("Error while closing dir");
		return res;
	}

	// At this point, session_nb is equal to the highest dir number.
	// Increment it to create new folder.
	session_nb++;
	return session_nb;
}

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
int lsdir(const char *path)
{
	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_ERR("Error opening dir %s [%d]", path, res);
		return res;
	}

	LOG_DBG("Listing dir %s ...", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			LOG_DBG("[DIR ] %s", entry.name);
		} else {
			LOG_DBG("[FILE] %s (size = %zu)", entry.name, entry.size);
		}
		count++;
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}

	return res;
}

bool create_some_entries(const char *base_path)
{
	char path[MAX_PATH];
	struct fs_file_t file;
	int base = strlen(base_path);

	fs_file_t_init(&file);

	if (base >= (sizeof(path) - SOME_REQUIRED_LEN)) {
		LOG_ERR("Not enough concatenation buffer to create file paths");
		return false;
	}

	LOG_INF("Creating some dir entries in %s", base_path);
	strncpy(path, base_path, sizeof(path));

	path[base++] = '/';
	path[base] = 0;
	strcat(&path[base], SOME_FILE_NAME);

	if (fs_open(&file, path, FS_O_CREATE) != 0) {
		LOG_ERR("Failed to create file %s", path);
		return false;
	}
	fs_close(&file);

	path[base] = 0;
	strcat(&path[base], SOME_DIR_NAME);

	if (fs_mkdir(path) != 0) {
		LOG_ERR("Failed to create dir %s", path);
		/* If code gets here, it has at least successes to create the
		 * file so allow function to return true.
		 */
	}
	return true;
}

int nicla_sd_create_session(){
	char path[MAX_PATH];
	int base = strlen(disk_mount_pt);

	int nb = get_session_nb(disk_mount_pt);
	if (nb < 0){
		LOG_ERR("Unable to get session number.");
		return nb;
	}
	int length = snprintf( NULL, 0, "%d", nb);
	char* nb_str = malloc(length + 1 + strlen(SESSION_DIR_NAME));
	snprintf(nb_str, length + 1 + strlen(SESSION_DIR_NAME), "%s%d", SESSION_DIR_NAME, nb);

	strncpy(path, disk_mount_pt, sizeof(path));

	path[base++] = '/';
	path[base] = 0;

	strcat(&path[base], nb_str);
	base += strlen(nb_str);
	free(nb_str);

	int res = fs_mkdir(path);
	if (res != 0) {
		LOG_ERR("Failed to create dir %s", path);
		return res;
	}

	fs_file_t_init(&current_session_acc_file);
	strcat(&path[base], "/"SESSION_ACC_FILE_NAME SESSION_FILE_EXTENSION);
	res = fs_open(&current_session_acc_file, path, FS_O_RDWR | FS_O_CREATE);
	if (res != 0) {
		LOG_ERR("Failed to create data_file %s (%i)", path, res);
		return res;
	}

	path[base] = 0;
	fs_file_t_init(&current_session_gyro_file);
	strcat(&path[base], "/"SESSION_GYRO_FILE_NAME SESSION_FILE_EXTENSION);
	res = fs_open(&current_session_gyro_file, path, FS_O_RDWR | FS_O_CREATE);
	if (res != 0) {
		LOG_ERR("Failed to create data_file %s (%i)", path, res);
		return res;
	}

	fs_write(&current_session_acc_file, SESSION_FILE_HEADER(SESSION_ACC_FILE_NAME), strlen(SESSION_FILE_HEADER(SESSION_ACC_FILE_NAME)));
	fs_write(&current_session_gyro_file, SESSION_FILE_HEADER(SESSION_GYRO_FILE_NAME), strlen(SESSION_FILE_HEADER(SESSION_GYRO_FILE_NAME)));

	current_session_nb = nb;

	return nb;
}

int nicla_sd_end_current_session(){
	int res = fs_close(&current_session_acc_file);
	if (res != 0) {
		LOG_WRN("Unable to close acc file (%i)", res);
		return res;
	}
	res = fs_close(&current_session_gyro_file);
	if (res != 0) {
		LOG_WRN("Unable to close gyro file (%i)", res);
		return res;
	}
	return 0;
}

int nicla_sd_unmount(){
	return fs_unmount(&mp);
}

void nicla_sd_init()
{

	/* raw disk i/o */
	do {
		static const char *disk_pdrv = DISK_DRIVE_NAME;
		uint64_t memory_size_mb;
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("Storage init ERROR!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			LOG_ERR("Unable to get sector count");
			break;
		}
		LOG_DBG("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			LOG_ERR("Unable to get sector size");
			break;
		}
		LOG_DBG("Sector size %u", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		LOG_DBG("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));
	} while (0);

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK) {
		LOG_DBG("Disk mounted.");
		if (lsdir(disk_mount_pt) == 0) {
			if (create_some_entries(disk_mount_pt)) {
				lsdir(disk_mount_pt);
			}
		}
	} else {
		LOG_DBG("Error mounting disk.");
	}
}
