/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ble_mpu.h"


/* Custom Service Variables */
static struct bt_uuid_128 mpu_service_uuid = 
                             BT_UUID_INIT_128( SERVICE_UUID );

static const struct bt_uuid_128 mpu_char_uuid =
                             BT_UUID_INIT_128( CHAR_UUID );

static const struct bt_uuid_128 mpu_write_char_uuid =
                             BT_UUID_INIT_128( WRITE_UUID );

static struct bt_gatt_ccc_cfg mpu_ccc_cfg[BT_GATT_CCC_MAX];

static struct bt_gatt_mpu_cb mpu_cb;

static volatile bool notifyEnable;

//User Descriptor
#define AXIS9_CUD          "Axis9"
#define WRITE_CUD          "Sensor Range"

static u8_t mpu_vals[BT_BUF];
static u8_t write_vals;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    notifyEnable = (value == BT_GATT_CCC_NOTIFY) ? true : false;
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        const void *buf, u16_t len, u16_t offset,
                        u8_t flags)
{

    if (mpu_cb.received_cb) {
      mpu_cb.received_cb(conn, buf, len);
    }

    return len;
}

/* GATT Attribute */
static struct bt_gatt_attr attrs[] = {
    /* Vendor Primary Service Declaration */
    BT_GATT_PRIMARY_SERVICE(&mpu_service_uuid),
    
    /* Notitication Characteristic */
    BT_GATT_CHARACTERISTIC(&mpu_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           NULL, NULL, mpu_vals),
                           //read_ct, write_ct, mpu_vals),
    BT_GATT_CUD(AXIS9_CUD, BT_GATT_PERM_READ), //Add User Descriptor
    BT_GATT_CCC(mpu_ccc_cfg, mpu_ccc_cfg_changed),

    /* Write Characteristic */
    BT_GATT_CHARACTERISTIC(&mpu_write_char_uuid.uuid,
                            BT_GATT_CHRC_WRITE,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                            NULL, write_ct, &write_vals),
     BT_GATT_CUD(WRITE_CUD, BT_GATT_PERM_READ), //Add User Descriptor
    
};

static struct bt_gatt_service mpu_svc = BT_GATT_SERVICE(attrs);


void ble_mpu_init(struct bt_gatt_mpu_cb *callbacks)
{
    if (callbacks) {
            mpu_cb.received_cb = callbacks->received_cb;
            //nus_cb.sent_cb     = callbacks->sent_cb;
    }
    bt_gatt_service_register(&mpu_svc);
}

bool ble_mpu_is_notify(void)
{
    return notifyEnable;
}

void ble_mpu_notify(struct bt_conn* conn, u8_t *p_vals, u16_t len)
{

    if(len > BT_BUF){
        return;
    }

    if(p_vals == NULL){
        return;
    }
    
    bt_gatt_notify(conn, &attrs[1], p_vals, BT_BUF);
}
