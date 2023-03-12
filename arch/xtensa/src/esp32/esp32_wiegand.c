/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wiegand.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/wiegand/wiegand.h>

#include <arch/board/board.h>

#include "chip.h"
#include "esp32_gpio.h"
#include "esp32_irq.h"

#include "xtensa.h"
#include "hardware/esp32_soc.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_pinmap.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "esp32_irq.h"
#include "esp32_wiegand.h"

#ifdef CONFIG_DRIVERS_WIEGAND

#ifndef CONFIG_ESP32_GPIO_IRQ
#error "This drive needs to enable config of CONFIG_ESP32_GPIO_IRQ," \
       " please enable it"
#endif

#if (CONFIG_WIEGAND_DATA_0_PIN == CONFIG_WIEGAND_DATA_1_PIN)
#error "The GPIOs  are the same, you should use different pins"
#endif

#define GPIO_WIEGAND_DATA_0 21
#define GPIO_WIEGAND_DATA_1 22


struct esp32_lowerhalf_s
{

    const struct wiegand_ops_s *ops; /* */

    spinlock_t lock; /* */

    xcpt_t isr; /* */

    void *arg; /* */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int esp32_setup(struct wiegand_lowerhalf_s *lower);
static int esp32_ioctl(struct wiegand_lowerhalf_s *lower, int cmd,
                       unsigned long arg);
static bool esp32_getdata(struct wiegand_lowerhalf_s *lower, int data);
static int esp32_interrupt(struct wiegand_lowerhalf_s *lower, xcpt_t isr, void *arg);
static int esp32_attach(struct wiegand_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structre */

static const struct wiegand_ops_s g_wiegandcallbacks = {
        .setup = esp32_setup,
        .getdata = esp32_getdata,
        .interrupt = esp32_interrupt,
        .ioctl = esp32_ioctl,
};


static struct esp32_lowerhalf_s g_wiegandlower = {
        .ops = &g_wiegandcallbacks,
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

/****************************************************************************
 * Name : esp32_setup
 *
 * Description:
 *
 *
 ****************************************************************************/
static int esp32_setup(struct wiegand_lowerhalf_s *lower)
{
    struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
    irqstate_t flags;
    int irq[2];

    flags = spin_lock_irqsave(&priv->lock);
    syslog(LOG_INFO, "Enable interrupt");
    
    esp32_configgpio(GPIO_WIEGAND_DATA_0, INPUT_FUNCTION_3 | PULLUP);
    esp32_configgpio(GPIO_WIEGAND_DATA_1, INPUT_FUNCTION_3 | PULLUP);

    if (priv->isr != NULL)
    {
        syslog(LOG_INFO, "Enable interrupt falling edge");

        irq[0] = ESP32_PIN2IRQ(GPIO_WIEGAND_DATA_0);
        irq[1] = ESP32_PIN2IRQ(GPIO_WIEGAND_DATA_1);

        esp32_gpioirqenable(irq[0], FALLING);
        esp32_gpioirqenable(irq[1], FALLING);
    }

    spin_unlock_irqrestore(&priv->lock, flags);

    return OK;
}

/****************************************************************************
 * Name : esp32_interrupt
 *
 * Description:
 *
 *
 ****************************************************************************/
static int esp32_interrupt(struct wiegand_lowerhalf_s *lower, xcpt_t isr, void *arg)
{
    struct esp32_lowerhalf_s *priv = (struct wiegand_lowerhalf_s *)lower;
    int irq [2];

    syslog(LOG_INFO, "Interrupt");
    irq[0] = ESP32_PIN2IRQ(GPIO_WIEGAND_DATA_0);
    irq[1] = ESP32_PIN2IRQ(GPIO_WIEGAND_DATA_1);

    if (isr != NULL)
    {
        esp32_gpioirqdisable(irq[0]);
        esp32_gpioirqdisable(irq[1]);

        int ret = irq_attach(irq[0], isr, arg);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: irq_attach() gpio DATA 0 failed: %d\n",
                   ret);
            return ret;
        }

        ret = irq_attach(irq[1], isr, arg);
        if (ret < 0)
        {
            syslog(LOG_ERR, "ERROR: irq_attach() gpio DATA 1 failed: %d\n",
                   ret);
            return ret;
        }

        esp32_gpioirqenable(irq[0], FALLING);
        esp32_gpioirqenable(irq[1], FALLING);
    }
    else
    {
        esp32_gpioirqdisable(irq[0]);
        esp32_gpioirqdisable(irq[1]);
    }

    priv->arg = arg;
    priv->isr = isr;

    return OK;
}

/****************************************************************************

 * Name : esp32_ioctl
 *
 * Description:
 *
 *
 ****************************************************************************/
static int esp32_ioctl(struct wiegand_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
    return -ENOTTY;
}

/****************************************************************************

 * Name : esp32_getdata
 *
 * Description:
 *
 *
 ****************************************************************************/
static bool esp32_getdata(struct wiegand_lowerhalf_s *lower, int data)
{
   struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
 
    if (data == 0)
    {
        return esp32_gpioread(GPIO_WIEGAND_DATA_0);
    }
    return esp32_gpioread(GPIO_WIEGAND_DATA_1);
}

/****************************************************************************
 * Name: esp32_wiegandlower
 *
 * Description:
 *   
 *
 ****************************************************************************/
static struct esp32_lowerhalf_s *esp32_wiegandlower(void)
{
    return &g_wiegandlower;
}


static int esp32_attach(struct wiegand_lowerhalf_s *lower)
{
    struct esp32_lowerhalf_s *priv = (struct esp32_lowerhalf_s *)lower;
    
    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wiegand_initialize
 *
 * Description:
 *   Initialize a wiegand interface.
 * 
 * Input Parameters:
 *  devpath - The full path to the driver to register.
 *
 * Returned Value:
 *  Zero on success, otherwise error is returned.
 ****************************************************************************/

int esp32_wiegand_initialize(char *devpath)
{
    struct esp32_lowerhalf_s *priv;

    priv = esp32_wiegandlower();
    if ( priv == NULL)
    {
        snerr("ERROR: Wiegand not configured\n");
        return -ENXIO;
    }

    int ret = wiegand_register(devpath, (struct wiegand_lowerhalf_s*)priv);
    if (ret < 0)
    {
        snerr("ERROR: wiegand_register falied: %d\n", ret);
        return ret;
    }

    return OK;
}

#endif