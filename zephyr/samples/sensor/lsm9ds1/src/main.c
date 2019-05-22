/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <sensor.h>
#include <stdio.h>

#include <lsm9ds1.h> // drivers/sensor/lsm9ds1

#include "MadgwickAHRS.h"


void main(void)
{
    struct device *dev = device_get_binding("LSM9DS1");

    if (dev == NULL) {
        printk("Could not get LSM9DS1 device\n");
        return;
    }

    printk("dev %p name %s\n", dev, dev->config->name);

    struct lsm9ds1_api* dev_api = (struct lsm9ds1_api *)dev->driver_api;
    
	while (1) {
		//struct sensor_value temp, press, humidity;
        float accel[3], gyro[3], mag[3];
        float temp;
        
        dev_api->sample_fetch(dev);

        dev_api->channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        dev_api->channel_get(dev, SENSOR_CHAN_GYRO_XYZ,  gyro);
        dev_api->channel_get(dev, SENSOR_CHAN_MAGN_XYZ,  mag);
        dev_api->channel_get(dev, SENSOR_CHAN_DIE_TEMP,  &temp);
        
        MadgwickAHRSupdate(deg2rad(gyro[0]), deg2rad(gyro[1]), deg2rad(gyro[2]), accel[0],accel[1],accel[2], -mag[0],mag[1],mag[2]);
        
        Quaternion q = getQuaternion();
        
        printf("Accel: %.6f, %.6f, %.6f\n", accel[0], accel[1], accel[2]);
        printf("Gyro:  %.6f, %.6f, %.6f\n", gyro[0],  gyro[1],  gyro[2]);
        printf("Mag:   %.6f, %.6f, %.6f\n", mag[0],   mag[1],   mag[2]);
        printf("temp:  %.2f\n", temp);
        printf("Quaternion:   %.6f, %.6f, %.6f, %6f\n", q.x, q.y, q.z, q.w);
        printf("\n");
        
		k_sleep(K_MSEC(1000));
	}
}
