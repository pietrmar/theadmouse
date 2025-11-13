#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>

#include <errno.h>
#include <stdio.h>

#include "button_manager.h"
#include "slot_manager.h"

#define MAX_PATH_LEN	255

LOG_MODULE_REGISTER(slot_manager, LOG_LEVEL_DBG);

static struct slot_settings active_slot;

// NOTE: This does not create directories recursively, but we
// do not want a deep hieararchy anyway as directories are
// expensive on LittleFS.
static inline int ensure_dir_exists(const char *path)
{
	struct fs_dirent entry;
	int ret = fs_stat(path, &entry);

	if (ret == 0) {
		if (entry.type != FS_DIR_ENTRY_DIR) {
			LOG_ERR("Path %s, exists but is not a directory", path);
			return -ENOTDIR;
		}
		// Path exists and is directory
		return 0;
	} else if (ret == -ENOENT) {
		// Path does not exist, create it
		LOG_DBG("Directory %s does not exist, creating it", path);
		return fs_mkdir(path);
	} else {
		// Some other error, like I/O or so
		return ret;
	}

	return 0;
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
static int file_printf(struct fs_file_t *f, const char *fmt, ...)
{
	char buf[256];

	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (n < 0) {
		return n; // Formatting error
	}

	if (n >= sizeof(buf)) {
		return -ENOMEM;
	}

	ssize_t bytes_written = fs_write(f, buf, (size_t)n);
	if (bytes_written < 0) {
		return (int)bytes_written;
	} else if (bytes_written != n) {
		return -EIO;
	}

	return (int)bytes_written;
}

static int read_line_from_file(char *buf, size_t len, struct fs_file_t *file)
{
	if (len == 0)
		return -EINVAL;

	// TODO: This does not handle truncating of lines if they are longer than `len`
	size_t i = 0;
	while (i < (len - 1)) {
		char c;
		ssize_t nread = fs_read(file, &c, 1);
		if (nread < 0)
			return (int)nread;

		if (nread == 0) {
			buf[i] = '\0';
			return EOF;
		}

		if (c == '\n')
			break;

		buf[i] = c;
		i++;
	}

	buf[i] = '\0';
	return 0;
}

int slot_manager_dump_all_slots(slot_printf_fn printf_fn)
{
	for (size_t i = 0; i < SLOT_MANAGER_MAX_SLOTS; i++){
		char buf[MAX_PATH_LEN + 1];

		int ret = snprintf(buf, sizeof(buf), "%s/slots_%02u/%02u",
					SLOT_MANAGER_BASE_PATH, SLOT_MANAGER_SCHEMA_VERSION, i);
		if (ret < 0 || ret >= sizeof(buf)) {
			ret = ret < 0 ? ret : -ENAMETOOLONG;

			LOG_ERR("Failed to format slot path: %d", ret);
			return ret;
		}


		struct fs_file_t f;
		fs_file_t_init(&f);

		ret = fs_open(&f, buf, FS_O_READ);
		if (ret == -ENOENT) {
			// If the file is not found then we've reached the last slot
			// and break out of the look.
			break;
		}

		// Other error
		if (ret < 0) {
			LOG_ERR("Failed to open %s: %d", buf, ret);
			return ret;
		}

		bool first_line = true;
		while (true) {
			ret = read_line_from_file(buf, sizeof(buf), &f);

			if (ret == EOF)
				break;

			if (ret < 0) {
				fs_close(&f);
				return ret;
			}

			if (first_line) {
				ret = printf_fn("Slot:%s", buf);
				first_line = false;
			} else {
				ret = printf_fn("%s", buf);
			}

			if (ret < 0) {
				fs_close(&f);
				return ret;
			}
		}

		fs_close(&f);
	}
	printf_fn("END");

	return 0;
}

// TODO: Currently the slots are saved in a file by serializing the AT commands
// that are needed to restore the slot state. In the future maybe we could use
// a more compact reprensentation in the flash instead of just duming the ASCII
// AT commands to a file.
static int save_current_slot(int idx, const char *name)
{
	if (idx < 0 || idx >= SLOT_MANAGER_MAX_SLOTS) {
		return -EINVAL;
	}

	char buf[MAX_PATH_LEN + 1];
	int ret = snprintf(buf, sizeof(buf), "%s/slots_%02u/%02u",
				SLOT_MANAGER_BASE_PATH, SLOT_MANAGER_SCHEMA_VERSION, idx);

	if (ret < 0 || ret >= sizeof(buf)) {
		ret = ret < 0 ? ret : -ENAMETOOLONG;

		LOG_ERR("Failed to format slot path: %d", ret);
		return ret;
	}

	struct fs_file_t f;
	fs_file_t_init(&f);

	ret = fs_open(&f, buf, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret < 0) {
		LOG_ERR("Failed to open %s: %d", buf, ret);
		return ret;
	}

	// Write the slot name
	file_printf(&f, "%s\n", name);

	// Serialize the current settings
	file_printf(&f, "AT SC 0x%06x\n", active_slot.led_color);

	// Serialize the current button configuration
	for (size_t i = 0; i < button_manager_get_num_buttons(); i++) {
		file_printf(&f, "AT BM %d\n", i + 1);

		uint16_t at_cmd;
		struct at_cmd_param *at_cmd_param;
		ret = button_manager_get_mapping(i, &at_cmd, &at_cmd_param);
		if (ret < 0) {
			// No button mapping exists
			file_printf(&f, "AT NC\n");
		} else {
			ret = at_format_cmd(buf, sizeof(buf), at_cmd, at_cmd_param);
			if (ret < 0) {
				LOG_WRN("Failed to format AT command: %d", ret);
				file_printf(&f, "AT NC\n");
			} else {
				file_printf(&f, "%s\n", buf);
			}
		}
	}

	ret = fs_close(&f);

	return ret;
}

static void reset_slot(struct slot_settings *slot)
{
	slot->led_color = 0x00000000;
}

int slot_manager_init(void)
{
	char file_path[MAX_PATH_LEN + 1];

	int ret = snprintf(file_path, sizeof(file_path), "%s/slots_%02u", SLOT_MANAGER_BASE_PATH, SLOT_MANAGER_SCHEMA_VERSION);
	if (ret < 0 || ret >= sizeof(file_path)) {
		ret = ret < 0 ? ret : -ENAMETOOLONG;

		LOG_ERR("Failed to format slot directory: %d", ret);
		return ret;
	}

	ret = ensure_dir_exists(file_path);
	if (ret < 0) {
		LOG_ERR("Failed to ensure directory at %s exists: %d", file_path, ret);
		return ret;
	}

	// TODO: For now create some dummy slots for testing in the filesystem
	active_slot.led_color = 0x00FF0000;
	save_current_slot(0, "test 0");
	active_slot.led_color = 0x0000FF00;
	save_current_slot(1, "test 1");
	active_slot.led_color = 0x000000FF;
	save_current_slot(2, "test 2");
	active_slot.led_color = 0x00FF0000;
	save_current_slot(3, "test 3");
	active_slot.led_color = 0x0000FF00;
	save_current_slot(4, "test 4");
	active_slot.led_color = 0x000000FF;
	save_current_slot(5, "test 5");
	active_slot.led_color = 0x00FF0000;
	save_current_slot(6, "test 6");
	active_slot.led_color = 0x0000FF00;
	save_current_slot(7, "test 7");

	return 0;
}
