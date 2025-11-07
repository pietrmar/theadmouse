#pragma once

int at_handle_line(char *s, uint16_t len);

int at_reply(const char *s);
int at_replyf(const char *fmt, ...);
