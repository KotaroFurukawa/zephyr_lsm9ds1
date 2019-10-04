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

static struct bt_gatt_ccc_cfg mpu_ccc_cfg[BT_GATT_CCC_MAX];

static volatile bool notifyEnable;

//User Descriptor
#define AXIS9_CUD          "Tx"

//extern u8_t* mpu_vals;
static u8_t mpu_vals[BT_BUF];

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
    notifyEnable = (value == BT_GATT_CCC_NOTIFY) ? true : false;
}

/* GATT Attribute */
static struct bt_gatt_attr attrs[] = {
    /* Vendor Primary Service Declaration */
    BT_GATT_PRIMARY_SERVICE(&mpu_service_uuid),
    
    BT_GATT_CHARACTERISTIC(&mpu_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           NULL, NULL, mpu_vals),
                           //read_ct, write_ct, mpu_vals),
    BT_GATT_CUD(AXIS9_CUD, BT_GATT_PERM_READ), //Add User Descriptor
    BT_GATT_CCC(mpu_ccc_cfg, mpu_ccc_cfg_changed),
    
};

static struct bt_gatt_service mpu_svc = BT_GATT_SERVICE(attrs);


void bmpu_init(void)
{
    bt_gatt_service_register(&mpu_svc);
}

bool bmpu_is_notify(void)
{
    return notifyEnable;
}

void bmpu_notify(struct bt_conn* conn, u8_t *p_vals, u16_t len)
{
    if(conn == NULL){
        return;
    }
    
    if(!notifyEnable){
        return;
    }
    
//    if(len > BT_BUF){
//        return;
//    }
    //if(p_vals == NULL){
    //    return;
    //}
    
//    memset(mpu_vals, 0, sizeof(mpu_vals));
//    memcpy(mpu_vals, p_vals, BT_BUF);
    
    bt_gatt_notify(conn, &attrs[1], p_vals, BT_BUF);
}
