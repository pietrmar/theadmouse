#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(theadmouse, CONFIG_THEADMOUSE_LOG_LEVEL);

int main(void)
{
	LOG_INF("Tina was here ^_^");

	while (true) {
		k_msleep(100);
	}

	return 0;
}
