/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_cc1101.c
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

#include <string.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/cc1101.h>

#include "stm32l4.h"
#include "nucleo-l476rg.h"

#ifdef CONFIG_WL_CC1101

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cc1101_wait
 *
 * Description:
 *
 ****************************************************************************/

static void cc1101_wait(struct cc1101_dev_s *dev, uint32_t pin)
{
  while (stm32l4_gpioread(pin) == true)
    {
    }
}

/****************************************************************************
 * Name: cc1101_irq
 *
 * Description:
 *
 ****************************************************************************/

static void cc1101_irq(struct cc1101_dev_s *dev, bool enable)
{
  if (enable)
    {
      stm32l4_gpiosetevent(dev->isr_pin, false, true, true, cc1101_isr, dev);
    }
  else
    {
      stm32l4_gpiosetevent(dev->isr_pin, false, true, true, NULL, NULL);
    }
}

/****************************************************************************
 * Name: cc1101_pwr
 *
 * Description:
 *
 ****************************************************************************/

static void cc1101_pwr(struct cc1101_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_cc1101_initialize
 *
 * Description:
 *   Initialize and register the cc1101 radio driver
 *
 ****************************************************************************/

int stm32l4_cc1101_initialize(void)
{
  struct spi_dev_s *spi    = NULL;
  struct cc1101_dev_s *dev = NULL;

  spi = stm32l4_spibus_initialize(CONFIG_CC1101_SPIDEV);
  if (spi == NULL)
    {
      ierr("ERROR: Failed to initialize SPI bus %d\n", CONFIG_CC1101_SPIDEV);
      return -ENODEV;
    }

  dev = kmm_malloc(sizeof(struct cc1101_dev_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(struct cc1101_dev_s));

  dev->spi        = spi;
  dev->isr_pin    = GPIO_CC1101_GDO2;
  dev->miso_pin   = GPIO_CC1101_MISO;
  dev->gdo        = CC1101_PIN_GDO2;
  dev->rfsettings = &cc1101_rfsettings_ISM2_433MHzMSK500kbps;
  dev->dev_id     = SPIDEV_WIRELESS(5);
  dev->channel    = 0;
  dev->power      = 1;
  dev->ops.wait   = cc1101_wait;
  dev->ops.pwr    = cc1101_pwr;
  dev->ops.irq    = cc1101_irq;

  return cc1101_register("/dev/cc1101", dev);
}
#endif
