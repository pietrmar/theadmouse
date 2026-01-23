#pragma once

#include "hm_hid_evt.h"

int hm_input_report_key(uint8_t hid_keycode, bool pressed, k_timeout_t timeout);
int hm_input_report_mouse_move(int16_t dx, int16_t dy, k_timeout_t timeout);
int hm_input_report_mouse_wheel(int8_t steps, k_timeout_t timeout);
int hm_input_report_mouse_btn(enum hm_hid_mouse_btn btn, bool pressed, k_timeout_t timeout);

const char *hm_input_get_current_kbd_layout_locale(void);
int hm_input_set_kbd_layout_locale(const char *locale);

int hm_input_write_string(const char *s);

int hm_input_icode_list_from_string(char *str, uint8_t *out, size_t len);
int hm_input_icode_to_hid(uint8_t icode);
int hm_input_icode_to_keyname(char *buf, size_t len, uint8_t icode);
