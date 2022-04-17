/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_gs2200m.c
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

#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/wireless/gs2200m.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include <arch/board/board.h>
#include "stm32f4discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_GS2200M_INT  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                           GPIO_OPENDRAIN | GPIO_PORTD | GPIO_PIN2)

#define GPIO_GS2200M_XRST (GPIO_OUTPUT | GPIO_PUSHPULL | \
                           GPIO_OUTPUT_SET | GPIO_SPEED_50MHz | \
                           GPIO_PORTE | GPIO_PIN4)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  gs2200m_irq_attach(xcpt_t, void *);
static void gs2200m_irq_enable(void);
static void gs2200m_irq_disable(void);
static uint32_t gs2200m_dready(int *);
static void gs2200m_reset(bool);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gs2200m_lower_s g_wifi_lower =
{
  .attach  = gs2200m_irq_attach,
  .enable  = gs2200m_irq_enable,
  .disable = gs2200m_irq_disable,
  .dready  = gs2200m_dready,
  .reset   = gs2200m_reset
};

static void *g_devhandle = NULL;
static volatile int32_t  _enable_count = 0;
static volatile uint32_t _n_called;

static xcpt_t g_irq_handler = NULL;
static void *g_irq_arg = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gs2200m_irq_attach
 ****************************************************************************/

static int gs2200m_irq_attach(xcpt_t handler, void *arg)
{
  /* NOTE: Just save the handler and arg here */

  g_irq_handler = handler;
  g_irq_arg = arg;
  return 0;
}

/****************************************************************************
 * Name: gs2200m_irq_enable
 ****************************************************************************/

static void gs2200m_irq_enable(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);
  uint32_t dready = 0;

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  if (0 == _enable_count)
    {
      /* Check if irq has been asserted */

      dready = gs2200m_dready(NULL);

      /* NOTE: stm32 does not support level-triggered irq */

      stm32_gpiosetevent(GPIO_GS2200M_INT, true, false,
                         true, g_irq_handler, g_irq_arg);
    }

  _enable_count++;

  spin_unlock_irqrestore(NULL, flags);

  if (dready)
    {
      /* Call g_irq_handler directly */

      wlinfo("== ** call irq handler **\n");
      g_irq_handler(0, NULL, g_irq_arg);
    }
}

/****************************************************************************
 * Name: gs2200m_irq_disable
 ****************************************************************************/

static void gs2200m_irq_disable(void)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  wlinfo("== ec:%" PRId32 " called=%" PRId32 "\n",
         _enable_count, _n_called++);

  _enable_count--;

  if (0 == _enable_count)
    {
      stm32_gpiosetevent(GPIO_GS2200M_INT, false, false,
                         false, NULL, NULL);
    }

  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: gs2200m_dready
 ****************************************************************************/

static uint32_t gs2200m_dready(int *ec)
{
  irqstate_t flags = spin_lock_irqsave(NULL);

  uint32_t r = stm32_gpioread(GPIO_GS2200M_INT);

  if (ec)
    {
      /* Copy enable count (just for debug) */

      *ec = _enable_count;
    }

  spin_unlock_irqrestore(NULL, flags);
  return r;
}

/****************************************************************************
 * Name: gs2200m_reset
 ****************************************************************************/

static void gs2200m_reset(bool reset)
{
  stm32_gpiowrite(GPIO_GS2200M_XRST, !reset);
}

/****************************************************************************
 * Name: spi_pincontrol
 *
 * Description:
 *   Configure SPI1 pins
 *
 ****************************************************************************/

static void _config_pin(void)
{
  stm32_configgpio(GPIO_GS2200M_XRST); /* Assign PD6 as XRST */
  stm32_configgpio(GPIO_GS2200M_INT);  /* Assign PD2 as IRQ */
  stm32_configgpio(GPIO_SPI3_SCK_1);   /* Assign PB3 as SPI3_SCK */
  stm32_configgpio(GPIO_SPI3_MISO_1);  /* Assign PB4 as SPI3_MISO */
  stm32_configgpio(GPIO_SPI3_MOSI_1);  /* Assign PB5 as SPI3_MOSI */
  stm32_configgpio(GPIO_GS2200M_CS);   /* Assign PD7 as chip select */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_gs2200m_initialize
 ****************************************************************************/

int stm32_gs2200m_initialize(const char *devpath, int bus)
{
  struct spi_dev_s *spi;

  wlinfo("Initializing GS2200M..\n");

  if (!g_devhandle)
    {
      /* Configure pin */

      _config_pin();

      /* Initialize spi device */

      spi = stm32_spibus_initialize(bus);

      if (!spi)
        {
          wlerr("ERROR: Failed to initialize spi%d.\n", bus);
          return -ENODEV;
        }

      g_devhandle = gs2200m_register(devpath, spi, &g_wifi_lower);

      if (!g_devhandle)
        {
          wlerr("ERROR: Failed to register gs2200m driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}
