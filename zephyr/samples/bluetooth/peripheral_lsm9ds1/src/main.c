/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
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

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>

#include <sensor.h>
#include <gpio.h>

#include <stdio.h>

#include <lsm9ds1.h> // drivers/sensor/lsm9ds1
#include "MadgwickAHRS.h"

#include "services/ble_mpu.h"


#define OUTPUT_BUF_SIZE sizeof(float)*14
static u8_t sensor_vals[OUTPUT_BUF_SIZE];

static struct device* dev_lsm9ds1;

static volatile bool isConnected = false;
static struct bt_conn* p_conn = NULL;
//static struct bt_gatt_exchange_params exchange_params;

//BLE Advertise
static volatile u8_t mfg_data[] = { 0x00, 0x00, 0xaa, 0xbb };

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 4),
    
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                  0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

//static void exchange_func(struct bt_conn *conn, u8_t err,
//                          struct bt_gatt_exchange_params *params)
//{
//    struct bt_conn_info info = {0};
//
//    printk("MTU exchange %s\n", err == 0 ? "successful" : "failed");
//
//    err = bt_conn_get_info(conn, &info);
//    if (info.role == BT_CONN_ROLE_MASTER) {
//
//    }
//}

static void connected(struct bt_conn *conn, u8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
    } else {
        printk("Connected\n");
        isConnected = true;
        p_conn = conn;
    }
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    isConnected = false;
    p_conn = NULL;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    printk("Bluetooth initialized\n");
    
    //ble service init.
    bmpu_init();
    
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
    
//    exchange_params.func = exchange_func;
//    err = bt_gatt_exchange_mtu(NULL, &exchange_params);
    
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    
    printk("Advertising successfully started\n");
}

// Sensor data
void update_sensor_data(struct device* dev)
{
    struct lsm9ds1_api* dev_api = (struct lsm9ds1_api *)dev->driver_api;
    
    //12, 24, 36, 40
    float accel[3], gyro[3], mag[3];
    float temp;
    float q[4];
    
    dev_api->sample_fetch(dev);
    
    dev_api->channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    dev_api->channel_get(dev, SENSOR_CHAN_GYRO_XYZ,  gyro);
    dev_api->channel_get(dev, SENSOR_CHAN_MAGN_XYZ,  mag);
    dev_api->channel_get(dev, SENSOR_CHAN_DIE_TEMP,  &temp);
    
    MadgwickAHRSupdate(deg2rad(gyro[0]), deg2rad(gyro[1]), deg2rad(gyro[2]), accel[0],accel[1],accel[2], -mag[0],mag[1],mag[2]);
    
    Quaternion qua = getQuaternion();
    q[0] = qua.x; q[1] = qua.y; q[2] = qua.z; q[3] = qua.w;
    
    printf("acc:   x: %.6f    y: %.6f    z: %.6f\n", accel[0], accel[1], accel[2]);
    printf("gyr:   x: %.6f    y: %.6f    z: %.6f\n", gyro[0], gyro[1], gyro[2]);
    printf("mag:   x: %.6f    y: %.6f    z: %.6f\n", mag[0], mag[1], mag[2]);
    printf("temp:     %.2f\n", temp);
    printf("qua:   x: %.6f    y: %.6f    z: %.6f    w: %.6f\n", qua.x, qua.y, qua.z, qua.w);

    memset(sensor_vals, 0, sizeof(sensor_vals));

    memcpy(&sensor_vals[sizeof(float)*0], accel, sizeof(accel));
    memcpy(&sensor_vals[sizeof(float)*3], gyro, sizeof(gyro));
    memcpy(&sensor_vals[sizeof(float)*6], mag, sizeof(mag));
    memcpy(&sensor_vals[sizeof(float)*9], &temp, sizeof(temp));
    memcpy(&sensor_vals[sizeof(float)*10], q, sizeof(q));
}


void main(void)
{
    /* Set LED pin as output */
    struct device* port0 = device_get_binding("GPIO_0");
    gpio_pin_configure(port0, 17, GPIO_DIR_OUT);
    
    // flash  LED
    gpio_pin_write(port0, 17, 0);
    k_sleep(K_MSEC(500));
    gpio_pin_write(port0, 17, 1);
    k_sleep(K_MSEC(500));
    
    // sensor
    dev_lsm9ds1 = device_get_binding("LSM9DS1");
    printk("dev %p name %s\n", dev_lsm9ds1, dev_lsm9ds1->config->name);
    
    memset(sensor_vals, 0, sizeof(sensor_vals));
    
    k_sleep(K_MSEC(500));
    
    // set up BLE
    int err;
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    bt_conn_cb_register(&conn_callbacks);
    
    while (1) {
        if(isConnected == true){
            if(bmpu_is_notify()){
                update_sensor_data(dev_lsm9ds1);
                bmpu_notify(p_conn, sensor_vals, OUTPUT_BUF_SIZE);
            }
        }
        k_sleep(K_MSEC(100));
    }
    
}
