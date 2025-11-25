#pragma once

#include <zephyr/bluetooth/bluetooth.h>

#include "hm_hid_evt.h"

int ble_hid_service_init(void);

int ble_hid_service_connected(struct bt_conn *conn);
int ble_hid_service_disconnected(struct bt_conn *conn);

int ble_hid_queue_evt(struct hm_hid_evt *evt, k_timeout_t timeout);
