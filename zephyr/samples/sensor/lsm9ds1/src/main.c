/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <sensor.h>
#include <stdio.h>

void main(void)
{
    struct device *dev = device_get_binding("LSM9DS1");

    if (dev == NULL) {
        printk("Could not get LSM9DS1 device\n");
        return;
    }

    printk("dev %p name %s\n", dev, dev->config->name);

	while (1) {
        float accel[3], gyro[3], mag[3];
        
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, (void *)&accel);
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, (void *)&gyro);
        sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, (void *)&mag);
        
        printf("Accel: %.6f, %.6f, %.6f\n", accel[0], accel[1], accel[2]);
        printf("Gyro:  %.6f, %.6f, %.6f\n", gyro[0],  gyro[1],  gyro[2]);
        printf("Mag:   %.6f, %.6f, %.6f\n", mag[0],   mag[1],   mag[2]);
        
		k_sleep(1000);
	}
}
