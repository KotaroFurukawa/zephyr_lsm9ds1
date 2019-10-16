/*
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <gpio.h>

#define PIN 21

static void chip_reset(struct device *gpio,
               struct gpio_callback *cb, u32_t pins)
{
//    const u32_t stamp = k_cycle_get_32();

//    printk("GPIO reset line asserted, device reset.\n");
//    printk("Bye @ cycle32 %u\n", stamp);

    NVIC_SystemReset();
}

static void reset_pin_wait_low(struct device *port, u32_t pin)
{
    int err;
    u32_t val;

    /* Wait until the pin is pulled low */
    do {
        err = gpio_pin_read(port, pin, &val);
    } while (err == 0 && val != 0);
}

static int reset_pin_configure(struct device *p0)
{
    int err;
    u32_t pin;
    struct device *port = NULL;

    static struct gpio_callback gpio_ctx;

    /* MCU interface pins 0-2 */
    if (IS_ENABLED(CONFIG_BOARD_nRF52_BLUESHELL_RESET_P0_21)) {
        port = p0;
        pin = PIN;
    }

    if (port == NULL) {
        return -EINVAL;
    }

    err = gpio_pin_configure(port, pin,
                 GPIO_DIR_IN | GPIO_INT | GPIO_PUD_PULL_DOWN |
                 GPIO_INT_ACTIVE_HIGH | GPIO_INT_EDGE);
    if (err) {
        //NVIC_SystemReset();
        return err;
    }

    gpio_init_callback(&gpio_ctx, chip_reset, BIT(pin));
    err = gpio_add_callback(port, &gpio_ctx);
    if (err) {
        return err;
    }

    err = gpio_pin_enable_callback(port, pin);
    if (err) {
        return err;
    }

    /* Wait until the pin is pulled low before continuing.
     * This lets the other side ensure that they are ready.
     */
//    LOG_INF("GPIO reset line enabled on pin %s.%02u, holding..", port == p0 ? "P0" : "P1", pin);

    reset_pin_wait_low(port, pin);

    return 0;
}

static int init(struct device *dev)
{
    int rc;
    int err;
    u32_t val = 0;
    struct device *p0;

    p0 = device_get_binding("GPIO_0");
    if (!p0) {
        return -EIO;
    }
    
    if (IS_ENABLED(CONFIG_BOARD_NRF52_BLUESHELL_RESET)) {
        rc = reset_pin_configure(p0);
        if (rc) {
            return -EIO;
        }
        
    }

//    LOG_INF("Board configured.");

    return 0;
}


SYS_INIT(init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
