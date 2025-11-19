#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(headmouse_input, LOG_LEVEL_DBG);

DEVICE_DEFINE(
	headmouse_input_kbd,
	"headmouse_input_kbd",
	NULL,
	NULL,
	NULL,
	NULL,
	POST_KERNEL,
	CONFIG_INPUT_INIT_PRIORITY,
	NULL
);

DEVICE_DEFINE(
	headmouse_input_mouse,
	"headmouse_input_mouse",
	NULL,
	NULL,
	NULL,
	NULL,
	POST_KERNEL,
	CONFIG_INPUT_INIT_PRIORITY,
	NULL
);

const struct device *const dev_headmouse_input_kbd = DEVICE_GET(headmouse_input_kbd);
const struct device *const dev_headmouse_input_mouse = DEVICE_GET(headmouse_input_mouse);

static void on_kbd_input(struct input_event *evt, void *user_data)
{
	LOG_DBG("Got event, device: %s, sync: %u, type: %u, code: %u (%#x), value: %d",
			evt->dev->name, evt->sync, evt->type, evt->code, evt->code, evt->value);
}
INPUT_CALLBACK_DEFINE(dev_headmouse_input_kbd, on_kbd_input, NULL);

static void on_mouse_input(struct input_event *evt, void *user_data)
{
	LOG_DBG("Got event, device: %s, sync: %u, type: %u, code: %u (%#x), value: %d",
			evt->dev->name, evt->sync, evt->type, evt->code, evt->code, evt->value);
}
INPUT_CALLBACK_DEFINE(dev_headmouse_input_mouse, on_mouse_input, NULL);
