/** @file
 *  @brief BAS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define AXIS9_BASE_UUID                    {{ 0xEF, 0xB1, 0x0F, 0xDB, 0x73, 0x0A, 0x44, 0xA9, 0xB9, 0x47, 0x23, 0x1A, 0x00, 0x00, 0x3D, 0x28 }} /**< Used vendor specific UUID. */
#define SERVICE_UUID        0xEF, 0xB1, 0x0F, 0xDB, 0x73, 0x0A, 0x44, 0xA9, 0xB9, 0x47, 0x23, 0x1A, 0x01, 0x00, 0x3D, 0x28
#define CHAR_UUID           0xEF, 0xB1, 0x0F, 0xDB, 0x73, 0x0A, 0x44, 0xA9, 0xB9, 0x47, 0x23, 0x1A, 0x02, 0x00, 0x3D, 0x28
#define WRITE_UUID          0xEF, 0xB1, 0x0F, 0xDB, 0x73, 0x0A, 0x44, 0xA9, 0xB9, 0x47, 0x23, 0x1A, 0x03, 0x00, 0x3D, 0x28

#define BT_BUF sizeof(float)*13

/** @brief Callback type for data received. */
typedef void (*mpu_received_cb_t)(struct bt_conn *conn,
				  const u8_t *const data, u16_t len);

/** @brief Pointers to the callback functions for service events. */
struct bt_gatt_mpu_cb {

        /** Callback for data received. **/
	mpu_received_cb_t received_cb;

        /** Callback for data sent. **/
	//nus_sent_cb_t     sent_cb;
};

void ble_mpu_init(struct bt_gatt_mpu_cb *callbacks);
bool ble_mpu_is_notify(void);
void ble_mpu_notify(struct bt_conn* conn, u8_t *p_vals, u16_t len);

#ifdef __cplusplus
}
#endif
