#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>

#include <errno.h>
#include <stdio.h>
#include <math.h>

#include "button_manager.h"
#include "motion_engine.h"
#include "slot_manager.h"

#define MAX_PATH_LEN	255

LOG_MODULE_REGISTER(slot_manager, CONFIG_SLOT_MANAGER_LOG_LEVEL);

// TODO: This is currently implemented similar like on the FABI where
// we store the AT commands in plain-text and each slot corresponds
// to a file. We should consider making the on-disk storage more compact
// and have some in-ram look-up table, to not need to walk through
// the file-system when we want to do trivial operations like
// check where a slot is stored and such.

static struct slot_settings active_slot;
static int active_slot_index = -1;

// NOTE: This is not thread save, but all slot operations, including getting the currently active
// slot should happen from the AT command thread.
int slot_manager_get_active_slot_idx(void)
{
	return active_slot_index;
}

int slot_manager_set_color(uint32_t col)
{
	active_slot.color = col;
	return 0;
}

int slot_manager_set_input_mode(enum input_mode mode)
{
	if (!input_mode_valid(mode))
		return -EINVAL;

	active_slot.input_mode = mode;

	// TODO: Implement this
	LOG_WRN("%s not implemented", __func__);

	return 0;
}

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

// Reads a line from file **not including** the newline
// TODO: Does not handle truncated lines that are longer than `len`
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

int slot_manager_get_next_free_index(void)
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


		struct fs_dirent entry;
		ret = fs_stat(buf, &entry);

		if (ret == -ENOENT)
			return i;

		if (ret < 0)
			return ret;
	}

	return -ENOSPC;
}

static int slot_manager_get_slot_index_by_name(const char *name)
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

		ret = read_line_from_file(buf, sizeof(buf), &f);

		// TODO: Is this the correct way to handle this?
		if (ret == EOF) {
			LOG_ERR("File for slot %d is empty", i);
			fs_close(&f);
			return ret;
		}

		if (ret < 0) {
			fs_close(&f);
			return ret;
		}

		fs_close(&f);

		if (strcmp(buf, name) == 0) {
			return i;
		}
	}

	return -ENOENT;
}

int slot_manager_list_all_slots(slot_printf_fn printf_fn)
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

		ret = read_line_from_file(buf, sizeof(buf), &f);

		// TODO: Is this the correct way to handle this?
		if (ret == EOF) {
			LOG_ERR("File for slot %d is empty", i);
			fs_close(&f);
			return ret;
		}

		if (ret < 0) {
			fs_close(&f);
			return ret;
		}

		ret = printf_fn("Slot%d:%s", i + 1, buf);

		if (ret < 0) {
			fs_close(&f);
			return ret;
		}

		fs_close(&f);
	}

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
static int save_current_slot_by_index(int idx, const char *name)
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
	file_printf(&f, "AT SC 0x%06x\n", active_slot.color);
	file_printf(&f, "AT MM %u\n", active_slot.input_mode);

	int sens_x = (int)round(motion_engine_get_hid_axis_sensitivity(AXIS_X));
	int sens_y = (int)round(motion_engine_get_hid_axis_sensitivity(AXIS_X));
	file_printf(&f, "AT AX %d\n", sens_x);
	file_printf(&f, "AT AY %d\n", sens_y);

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

int slot_manager_save_current_slot_by_name(const char *name)
{
	int ret = slot_manager_get_slot_index_by_name(name);
	if (ret < 0 && ret != -ENOENT)
		return ret;

	if (ret == -ENOENT) {
		ret = slot_manager_get_next_free_index();
		if (ret < 0) {
			LOG_WRN("Failed to get the next free slot index: %d", ret);
			return ret;
		}
	}

	return save_current_slot_by_index(ret, name);
}


// HACK: This is just for quick testing
extern int led_set_rgb(int r, int g, int b);

static int __slot_manager_load_slot_by_index_nolock(int idx)
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

	ret = fs_open(&f, buf, FS_O_READ);
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
			LOG_INF("Loading slot: %s", buf);
			first_line = false;
		} else {
			const struct at_cmd *cmd = NULL;
			struct at_cmd_param cmd_param = { 0 };

			LOG_DBG("Parsing line: <%s>", buf);
			int ret = at_parse_line_inplace(buf, &cmd, &cmd_param, 0);
			if (ret < 0) {
				LOG_ERR("Failed to parse line: %d", ret);
				continue;
			}

			ret = at_cmd_dispatch_ptr(cmd, &cmd_param);
			if (ret < 0) {
				LOG_ERR("Failed to dispatch line: %d", ret);
				continue;
			}
		}
	}

	fs_close(&f);

	active_slot_index = idx;

	int r = (active_slot.color >> 16) & 0xFF;
	int g = (active_slot.color >>  8) & 0xFF;
	int b = (active_slot.color >>  0) & 0xFF;

	// Scale from 0-255 to a percentage value of 0-100 as this
	// is what the `led_set_rgb()` API expects.
	r = (r * 100 + 127) / 255;
	g = (g * 100 + 127) / 255;
	b = (b * 100 + 127) / 255;

	// This is also just a quick HACK here
	ret = led_set_rgb(r, g, b);

	return ret;
}

K_SEM_DEFINE(slot_manager_load_lock, 1, 1);
int slot_manager_load_slot_by_index(int idx)
{
	// When a slot is loaded all the commands inside the file are executed directly and not
	// enqueued on the command queue. If a settings file contains a slot loading command like
	// `AT NE` or `AT LO <slotname>` then this would lead to recursive slot loading and for
	// sure to some issues. Currently it should be impossible for the settings file to contain
	// such commands, but just in case this ever happens we just prevent it here with this
	// semaphore.
	if (k_sem_take(&slot_manager_load_lock, K_NO_WAIT) != 0) {
		LOG_ERR("Recursive slot loading detected!");
		return -EBUSY;
	}

	int ret = __slot_manager_load_slot_by_index_nolock(idx);

	k_sem_give(&slot_manager_load_lock);

	return ret;
}

int slot_manager_load_slot_by_name(const char *name)
{
	int ret = slot_manager_get_slot_index_by_name(name);
	if (ret < 0)
		return ret;

	return slot_manager_load_slot_by_index(ret);
}

int slot_manager_load_next_slot(void)
{
	int next_free = slot_manager_get_next_free_index();

	if (next_free < 0) {
		// If there was no more free slot (`-ENOSPC`) then wrap at `SLOT_MANAGER_MAX_SLOTS`.
		// If we got some other error code then error out.
		if (next_free == -ENOSPC)
			next_free = SLOT_MANAGER_MAX_SLOTS;
		else {
			LOG_ERR("slot_manager_get_next_free_index() failed: %d", next_free);
			return next_free;
		}
	}

	if (next_free == 0) {
		LOG_ERR("No slots configured");
		return -ENOENT;
	}

	if (active_slot_index < 0 || active_slot_index >= next_free)  {
		LOG_ERR("active_slot_index is invalid: %d", active_slot_index);
		return -EFAULT;
	}


	int new_index = (active_slot_index + 1) % next_free;

	LOG_INF("Loading next slot with index: %d", new_index);

	return slot_manager_load_slot_by_index(new_index);
}

static void reset_active_slot(void)
{
	active_slot.color = 0x00000000;
	active_slot.input_mode = INPUT_MODE_MOUSE;
	// TODO: Also maybe call into button manager to reset all mappings
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

	ret = slot_manager_get_next_free_index();
	if (ret < 0 && ret != -ENOSPC) {
		LOG_ERR("Failed to get next free slot index: %d", ret);
		return ret;
	}

	if (ret == 0) {
		LOG_INF("Creating empty default slot");

		reset_active_slot();

		ret = save_current_slot_by_index(0, "default");
		if (ret < 0) {
			LOG_ERR("Failed to create empty default slot: %d", ret);
			return ret;
		}
	}


	// Manually constuct and enqueue internal AT command that loads a slot by
	// index. This makes sure that `slot_manager_load_slot_by_index()` will be
	// called from the AT command execution thread.
	struct at_cmd_param param;
	ret = at_cmd_param_set_uint(&param, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set parameter: %d", ret);
		return ret;
	}

	ret = at_cmd_enqueue_code(AT_CMD_INTERNAL_LOAD_SLOT_INDEX, &param, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to enqueue command for loading slot 0: %d", ret);
		return ret;
	}

	return 0;
}
