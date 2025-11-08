
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include "at_cmd.h"
#include "button_manager.h"

LOG_MODULE_REGISTER(button_manager, LOG_LEVEL_DBG);

static const struct device *const btn_dev = DEVICE_DT_GET(DT_CHOSEN(mpi_buttons));

// TODO: We could just use an invalid/impossible `at_code` of `0x00` to indicate an
// invalid mapping to save space, but I think for now one simple valid flag is fine.
struct button_mapping {
	uint16_t code;
	bool valid;

	uint16_t at_code;
	struct at_cmd_param *at_param;
};

// TODO: Access to this table needs to be locked or some clever copy-on-write mechanism
static struct button_mapping btn_map[] = {
	{ .code = INPUT_BTN_0, .valid = false },
	{ .code = INPUT_BTN_1, .valid = false },
	{ .code = INPUT_BTN_2, .valid = false },
	{ .code = INPUT_BTN_3, .valid = false },
};

const size_t button_manager_num_buttons = ARRAY_SIZE(btn_map);

int button_manager_set_mapping(size_t idx, const uint16_t at_code, const struct at_cmd_param *at_param)
{
	char buf[3];

	if (!button_manager_valid_index(idx)) {
		LOG_WRN("Button index %d out of range", idx);
		return -EINVAL;
	}

	// TODO: Log the parameter also nicely here
	LOG_DBG("Setting button mapping for index %d, cmd: <%s>", idx, at_code_to_str(at_code, buf));

	struct button_mapping *mapping = &btn_map[idx];

	if (mapping->valid) {
		at_cmd_param_free(mapping->at_param);
		mapping->at_param = NULL;
		mapping->valid = false;
	}

	mapping->at_code = at_code;
	mapping->at_param = at_cmd_param_clone(at_param);
	if (!mapping->at_param) {
		LOG_ERR("failed to clone AT cmd params");
		return -ENOMEM;
	}

	mapping->valid = true;

	return 0;
}

int button_manager_delete_mapping(size_t idx)
{
	if (!button_manager_valid_index(idx)) {
		LOG_WRN("Button index %d out of range", idx);
		return -EINVAL;
	}

	LOG_DBG("Deleting button mapping for index %d", idx);

	struct button_mapping *mapping = &btn_map[idx];

	if (!mapping->valid)
		return 0;

	mapping->valid = false;
	at_cmd_param_free(mapping->at_param);
	mapping->at_param = NULL;

	return 0;
}

static void on_input(struct input_event *evt, void *user_data)
{
	LOG_DBG("Got event, device: %s, type: %u, code: %u (%#x), value: %u",
			evt->dev->name, evt->type, evt->code, evt->code, evt->value);

	if (evt->type != INPUT_EV_KEY)
		return;

	// TODO: For now we only care about down presses
	if (evt->value == 0)
		return;

	struct button_mapping *mapping = NULL;

	for (int i = 0; i < ARRAY_SIZE(btn_map); i++) {
		if (btn_map[i].code == evt->code) {
			mapping = &btn_map[i];
		}
	}

	if (mapping == NULL || !mapping->valid) {
		LOG_WRN("No valid entry found for key code %u (%#x)", evt->code, evt->code);
		return;
	}

	// TODO: Right now `at_dispatch_cmd()` can block, so we should not call it from here
	// and instead enqueue command ... or rewrite `at_dispatch_cmd()` to not block.
	int ret = at_dispatch_cmd(mapping->at_code, mapping->at_param);
	if (ret < 0) {
		LOG_ERR("Failed to dispatch AT command: %d", ret);
	}
}
INPUT_CALLBACK_DEFINE(btn_dev, on_input, NULL);

int button_manager_init(void)
{
	return 0;
}
