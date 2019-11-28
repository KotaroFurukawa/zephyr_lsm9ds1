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

#include <lsm9ds1.h> // drivers/sensor/lsm9ds1

#include "MadgwickAHRS.h"
#include "services/ble_mpu.h"

/* change this to use another GPIO port */
#ifndef SW0_GPIO_CONTROLLER
#ifdef SW0_GPIO_NAME
#define SW0_GPIO_CONTROLLER SW0_GPIO_NAME
#else
#error SW0_GPIO_NAME or SW0_GPIO_CONTROLLER needs to be set in board.h
#endif
#endif
#define SW0_PORT    SW0_GPIO_CONTROLLER

/* change this to use another GPIO pin */
#ifdef SW0_GPIO_PIN
#define BUTTON_PIN     SW0_GPIO_PIN
#else
#error SW0_GPIO_PIN needs to be set in board.h
#endif

/* change to use another GPIO pin interrupt config */
#ifdef SW0_GPIO_FLAGS
#define EDGE    (SW0_GPIO_FLAGS | GPIO_INT_EDGE)
#else
/*
 * If SW0_GPIO_FLAGS not defined used default EDGE value.
 * Change this to use a different interrupt trigger
 */
#define EDGE    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#endif

/* change this to enable pull-up/pull-down */
#ifndef SW0_GPIO_FLAGS
#ifdef SW0_GPIO_PIN_PUD
#define SW0_GPIO_FLAGS SW0_GPIO_PIN_PUD
#else
#define SW0_GPIO_FLAGS 0
#endif
#endif
#define PULL_UP SW0_GPIO_FLAGS

/* Button */
struct device *button_gpio;

/* LED */
#define LED_PORT LED0_GPIO_CONTROLLER
#define LED    LED0_GPIO_PIN
static struct device* dev_lsm9ds1;

//bt
static struct bt_conn* p_conn = NULL;

#define OUTPUT_BUF_SIZE BT_BUF
u8_t mpu_vals[OUTPUT_BUF_SIZE];

//BLE Advertise
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, SERVICE_UUID),
};


/* Interval in milliseconds between each time status LEDs are updated. */
#define LEDS_UPDATE_DELAY            500

/* Structures for work */
static struct k_delayed_work leds_update_work;


/* LED */
#define LED_PORT LED0_GPIO_CONTROLLER
#define LED    LED0_GPIO_PIN

static struct device *led_dev;

/**@brief Update LEDs state. */

static void led_one_shot(u8_t ms)
{
      gpio_pin_write(led_dev, LED, 1);
      k_sleep(K_MSEC(ms));
      gpio_pin_write(led_dev, LED, 0);

}

static void leds_update(struct k_work *work)
{

    // flash  LED
    while(true){
      led_one_shot(25);
      k_sleep(K_MSEC(5000));
    }

}

void button_pressed(struct device *gpiob, struct gpio_callback *cb,
            u32_t pins)
{
//    printk("Button pressed at %d\n", k_cycle_get_32());
}

static struct gpio_callback button_cb;



// Sensor data
static void sensor_update(void)
{

    float accel[3], gyro[3], mag[3], gyro_rad[3], q[4];

    struct lsm9ds1_api* dev_api = (struct lsm9ds1_api *)dev_lsm9ds1->driver_api;
    dev_api->sample_fetch(dev_lsm9ds1);
    
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_ACCEL_XYZ, accel);
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_GYRO_XYZ,  gyro);
    dev_api->channel_get(dev_lsm9ds1, SENSOR_CHAN_MAGN_XYZ,  mag);
//    dev_api->channel_get(dev, SENSOR_CHAN_DIE_TEMP,  &temp);
    
    for(int i = 0; i < 3; i ++){
        gyro_rad[i] = gyro[i] * 3.1415926535 / 180;
    }

    MadgwickAHRSupdate(gyro_rad[0], gyro_rad[1], gyro_rad[2],
                       accel[0], accel[1], accel[2],
                       mag[0], mag[1], mag[2]);

//    printf("acc:   x: %.6f    y: %.6f    z: %.6f\n", accel[0], accel[1], accel[2]);
//    printf("gyr:   x: %.6f    y: %.6f    z: %.6f\n", gyro[0], gyro[1], gyro[2]);
//    printf("mag:   x: %.6f    y: %.6f    z: %.6f\n", mag[0], mag[1], mag[2]);
//    printf("temp:     %.2f\n", temp);
//    printf("qua:   x: %.6f    y: %.6f    z: %.6f    w: %.6f\n", qua.x, qua.y, qua.z, qua.w);

    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;

    memcpy(&mpu_vals[sizeof(float)*0], accel, sizeof(accel));
    memcpy(&mpu_vals[sizeof(float)*3], gyro, sizeof(gyro));
    memcpy(&mpu_vals[sizeof(float)*6], mag, sizeof(mag));
    memcpy(&mpu_vals[sizeof(float)*9], q, sizeof(q));

}

static void received_cb(struct bt_conn *conn,
                          const u8_t *const data, u16_t len)
{

    if(len > 1){
        return;
    }
    
    u8_t value = 0;
    memcpy(&value, data, sizeof(u8_t));
    
    lsm9ds1_perform performance;

    switch(value){
       case 0:
         performance = LOW;
         break;
       case 1:
         performance = MID;
         break;
       case 2:
         performance = HIGH;
         break;
       default:
         performance = LOW;
         break;
    }

    struct lsm9ds1_api* dev_api = (struct lsm9ds1_api *)dev_lsm9ds1->driver_api;
    dev_api->sensor_performance(dev_lsm9ds1, performance);
}

static struct bt_gatt_mpu_cb mpu_cb = {
     .received_cb = received_cb,
};


static void connected(struct bt_conn *conn, u8_t err)
{
    if (!err) {
//        printk("Connected\n");
        p_conn = conn;
    }
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
//    printk("Disconnected (reason %u)\n", reason);
    
    p_conn = NULL;
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};


static void bt_ready(int err)
{
    if (err) {
//        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
//    printk("Bluetooth initialized\n");
    
    //ble service init.
    ble_mpu_init(&mpu_cb);
    
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
    
    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
//        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    
//    printk("Advertising successfully started\n");

}

static void work_init(void)
{

    k_delayed_work_init(&leds_update_work, leds_update);
    k_delayed_work_submit(&leds_update_work, LEDS_UPDATE_DELAY);
}

static void bt_init(void)
{
    // set up BLE
    int err;
    err = bt_enable(bt_ready);
    if (err) {
//        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    
    bt_conn_cb_register(&conn_callbacks);

}

static void sensor_init(void)
{
//    printk("Sensor init...\n");
    dev_lsm9ds1 = device_get_binding("LSM9DS1");

    struct lsm9ds1_api* dev_api = (struct lsm9ds1_api *)dev_lsm9ds1->driver_api;
    dev_api->init_done(dev_lsm9ds1);

//    memset(mpu_vals, 0, sizeof(mpu_vals));

}

/**@brief Initializes buttons and LEDs, using the DK buttons and LEDs
 * library.
 */
static void leds_init(void)
{

    led_dev = device_get_binding(LED_PORT);
    
    /* Set LED pin as output */
    gpio_pin_configure(led_dev, LED, GPIO_DIR_OUT);

}

static void button_init(void)
{
//    printk("Press the user defined button on the board\n");
    button_gpio = device_get_binding(SW0_PORT);
    if (!button_gpio) {
//        printk("error\n");
        return;
    }

    gpio_pin_configure(button_gpio, BUTTON_PIN,
               GPIO_DIR_IN | GPIO_INT |  PULL_UP | EDGE);

    gpio_init_callback(&button_cb, button_pressed, BIT(BUTTON_PIN));

    gpio_add_callback(button_gpio, &button_cb);
    gpio_pin_enable_callback(button_gpio, BUTTON_PIN);
}

void main(void)
{

//    k_sleep(K_SECONDS(10));
    
    button_init();
    leds_init();
    
    led_one_shot(200);
    sensor_init();
    led_one_shot(200);

    bt_init();
    work_init();

    while ( true ) {

      if(p_conn != NULL){
        if(ble_mpu_is_notify()){
            sensor_update();
            ble_mpu_notify(p_conn, mpu_vals, OUTPUT_BUF_SIZE);
        }
      }

      /* Put CPU to idle to save power */
      k_cpu_idle();
    }
    
}

