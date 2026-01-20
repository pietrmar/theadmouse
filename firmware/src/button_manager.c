
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include "at_cmd.h"
#include "button_manager.h"

LOG_MODULE_REGISTER(button_manager, CONFIG_BUTTON_MANAGER_LOG_LEVEL);

static const struct device *const btn_dev = DEVICE_DT_GET(DT_CHOSEN(mpi_buttons));

K_HEAP_DEFINE(btn_at_param_heap, 1024);

// TODO: We could just use an invalid/impossible `at_code` of `0x00` to indicate an
// invalid mapping to save space, but I think for now one simple valid flag is fine.
// TODO: Maybe consider just storing the code + a callback or so and deal with the
// AT command internals somewhere else.
struct button_mapping {
	uint16_t code;
	bool valid;

	uint16_t at_code;
	struct at_cmd_param at_param;
};

// TODO: Access to this table needs to be locked or some clever copy-on-write mechanism
static struct button_mapping btn_map[] = {
	{ .code = INPUT_BTN_0, .valid = false },
	{ .code = INPUT_BTN_1, .valid = false },
	{ .code = INPUT_BTN_2, .valid = false },
	{ .code = INPUT_BTN_3, .valid = false },
};

const size_t button_manager_num_buttons = ARRAY_SIZE(btn_map);

int button_manager_get_mapping(size_t idx, uint16_t *at_code, struct at_cmd_param **at_param)
{
	if (!button_manager_valid_index(idx)) {
		LOG_WRN("Button index %d out of range", idx);
		return -EINVAL;
	}

	struct button_mapping *mapping = &btn_map[idx];
	if (!mapping->valid) {
		return -ENOENT;
	}

	*at_code = mapping->at_code;
	// TODO: Consider returning a clone with the heap string to prevent race conditions, but we
	// need to check where `button_manager_get_mapping()` is used.
	*at_param = &mapping->at_param;

	return 0;
}

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
		at_cmd_param_free(&mapping->at_param, &btn_at_param_heap);
		mapping->valid = false;
	}

	mapping->at_code = at_code;
	int ret = at_cmd_param_clone(&mapping->at_param, at_param, &btn_at_param_heap);
	if (ret < 0) {
		LOG_ERR("failed to clone AT cmd param: %d", ret);
		return ret;
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
	at_cmd_param_free(&mapping->at_param, &btn_at_param_heap);

	return 0;
}

static size_t get_button_index(uint16_t code)
{
	for (int idx = 0; idx < ARRAY_SIZE(btn_map); idx++) {
		if (btn_map[idx].code == code)
			return idx;
	}

	return -ENOENT;
}

static atomic_t button_state = ATOMIC_INIT(0);

char *button_manager_get_button_state_string(char *buf, size_t size)
{
	atomic_val_t cur_state = atomic_get(&button_state);

	size_t i;
	for (i = 0; i < button_manager_num_buttons; i++) {
		if (i >= (size - 1))
			break;

		buf[i] = cur_state & BIT(i) ? '1' : '0';
	}

	buf[i] = 0;

	return buf;
}

static void on_input(struct input_event *evt, void *user_data)
{
	LOG_DBG("Got event, device: %s, sync: %u, type: %u, code: %u (%#x), value: %d",
			evt->dev->name, evt->sync, evt->type, evt->code, evt->code, evt->value);

	if (evt->type != INPUT_EV_KEY)
		return;

	int idx = get_button_index(evt->code);
	if (idx < 0) {
		LOG_WRN("No mapping entry for key code %u (%#x)", evt->code, evt->code);
		return;
	}

	// NOTE: For now we only care about down presses for dispatching AT commands, so just
	// clear the button_state bit and return here
	if (evt->value == 0) {
		atomic_clear_bit(&button_state, idx);
		return;
	}

	if (evt->value == 1) {
		atomic_set_bit(&button_state, idx);
	}

	struct button_mapping *mapping = &btn_map[idx];

	if (!mapping->valid) {
		LOG_WRN("Mapping is not valid for key code %u (%#x)", evt->code, evt->code);
		return;
	}

	// TODO: `at_cmd_enqueue_code()` will do a copy of the param struct, but unfortunately
	// not a deep copy, so we need to soemhow handle this in the futre in case we make
	// full/proper use of heap strings.
	int ret = at_cmd_enqueue_code(mapping->at_code, &mapping->at_param, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to enqueue AT command: %d", ret);
	}
}
INPUT_CALLBACK_DEFINE(btn_dev, on_input, NULL);

int button_manager_init(void)
{
	return 0;
}
