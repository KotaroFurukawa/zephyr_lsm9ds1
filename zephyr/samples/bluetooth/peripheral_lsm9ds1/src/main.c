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
//#include <misc/printk.h>
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
#include <dk_buttons_and_leds.h>

//#include <stdio.h>

#include <lsm9ds1.h> // drivers/sensor/lsm9ds1
#include "quaternionFilter.h"
#include "services/ble_mpu.h"

//IMU Quaternion value.
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float deltat;                            // integration interval for both filter schemes
uint32_t lastUpdate;
uint32_t Now;
float pitch, yaw, roll;
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

#define OUTPUT_BUF_SIZE BT_BUF
u8_t mpu_vals[OUTPUT_BUF_SIZE];

//sensor
static struct device*      dev_lsm9ds1;
static struct lsm9ds1_api* dev_api;

//bt
static struct bt_conn* p_conn = NULL;

//BLE Advertise
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SERVICE_UUID),
};

/* Interval in milliseconds between each time status LEDs are updated. */
#define LEDS_UPDATE_INTERVAL            500
#define BUTTONS_UPDATE_INTERVAL         500

// Timer
static volatile uint32_t overflows = 0;

/* Structures for work */
static struct k_delayed_work leds_update_work;
static struct k_delayed_work buttons_update_work;

static u32_t micros(void)
{

   u64_t ticks = (u64_t)((u64_t)overflows << (u64_t)24) | (u64_t)k_uptime_get();

   return (ticks * 1000000) / 32768;
}

// Sensor data
static void sensor_update(void)
{

    float accel[3], gyro[3], mag[3];

    dev_api->sample_fetch(dev_lsm9ds1);
    
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_ACCEL_XYZ, accel);
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_GYRO_XYZ,  gyro);
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_MAGN_XYZ,  mag);
//    dev_api->channel_get(dev, SENSOR_CHAN_DIE_TEMP,  &temp);
    
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    MadgwickQuaternionUpdate( accel[0], accel[1], accel[2],
                              gyro[0]*PI/180.0f, gyro[1]*PI/180.0f, gyro[2]*PI/180.0f,
                             -mag[0], mag[1], mag[2] );

//    printf("acc:   x: %.6f    y: %.6f    z: %.6f\n", accel[0], accel[1], accel[2]);
//    printf("gyr:   x: %.6f    y: %.6f    z: %.6f\n", gyro[0], gyro[1], gyro[2]);
//    printf("mag:   x: %.6f    y: %.6f    z: %.6f\n", mag[0], mag[1], mag[2]);
//    printf("temp:     %.2f\n", temp);
//    printf("qua:   x: %.6f    y: %.6f    z: %.6f    w: %.6f\n", qua.x, qua.y, qua.z, qua.w);


    memcpy(&mpu_vals[sizeof(float)*0], accel, sizeof(accel));
    memcpy(&mpu_vals[sizeof(float)*3], gyro, sizeof(gyro));
    memcpy(&mpu_vals[sizeof(float)*6], mag, sizeof(mag));
//    memcpy(&sensor_vals[sizeof(float)*9], &temp, sizeof(temp));
    memcpy(&mpu_vals[sizeof(float)*9], q, sizeof(q));

}

/**@brief Update BUTTONs state. */
static void buttons_update(struct k_work *work)
{

    bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

}

/**@brief Update LEDs state. */
static void leds_update(struct k_work *work)
{

    // flash  LED
    while(true){

      dk_set_led(DK_LED1, 1);
      k_sleep(K_MSEC(25));
      dk_set_led(DK_LED1, 0);

      k_sleep(K_MSEC(5000));

    }

}


static void connected(struct bt_conn *conn, u8_t err)
{
    if (err) {
        //printk("Connection failed (err %u)\n", err);
    } else {
        //printk("Connected\n");
        
        p_conn = conn;
    }
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
    //printk("Disconnected (reason %u)\n", reason);
    
    p_conn = NULL;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err)
{
    if (err) {
        //printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    //printk("Bluetooth initialized\n");
    
    //ble service init.
    bmpu_init();
    
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
    
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        //printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    
    //printk("Advertising successfully started\n");
}



static void work_init(void)
{

    k_delayed_work_init(&leds_update_work, leds_update);
    k_delayed_work_init(&buttons_update_work, buttons_update);

    k_delayed_work_submit(&leds_update_work, LEDS_UPDATE_INTERVAL);
    k_delayed_work_submit(&buttons_update_work, BUTTONS_UPDATE_INTERVAL);

}

static void bt_init(void)
{
    // set up BLE
    int err;
    err = bt_enable(bt_ready);
    if (err) {
        //printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    bt_conn_cb_register(&conn_callbacks);

}

static void sensor_init(void)
{

    //printk("dev %p name %s\n", dev_lsm9ds1, dev_lsm9ds1->config->name);
    dev_lsm9ds1 = device_get_binding("LSM9DS1");
    memset(mpu_vals, 0, sizeof(mpu_vals));

    dev_api = (struct lsm9ds1_api *)dev_lsm9ds1->driver_api;
    dev_api->sensor_performance(dev_lsm9ds1, true);


}

static void button_handler(u32_t button_state, u32_t has_changed)
{
    
    if (has_changed & DK_BTN1_MSK) {
        k_delayed_work_submit(&buttons_update_work, BUTTONS_UPDATE_INTERVAL);
    }
}

/**@brief Initializes buttons and LEDs, using the DK buttons and LEDs
 * library.
 */
static void buttons_leds_init(void)
{
    int err;
    
    err = dk_buttons_init(button_handler);
    if (err) {
        //printk("Could not initialize buttons, err code: %d\n", err);
    }
    
    err = dk_leds_init();
    if (err) {
        //printk("Could not initialize leds, err code: %d\n", err);
    }
    
    dk_set_led(DK_LED1, 1);
    k_sleep(200);
    dk_set_led(DK_LED1, 0);

    err = dk_set_leds_state(0x00, DK_ALL_LEDS_MSK);
    if (err) {
       // printk("Could not set leds state, err code: %d\n", err);
    }
}

void main(void)
{

    buttons_leds_init();
    sensor_init();

    bt_init();
    work_init();
    
    while ( true ) {

      if(p_conn != NULL){
        if(bmpu_is_notify()){

          sensor_update();
          bmpu_notify(p_conn, mpu_vals, OUTPUT_BUF_SIZE);
          k_sleep(K_MSEC(100));

        }
      }

      /* Put CPU to idle to save power */
     
      k_cpu_idle();
    }
    
}
