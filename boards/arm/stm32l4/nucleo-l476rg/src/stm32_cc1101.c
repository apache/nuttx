/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_cc1101.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: lihaichen <li8303@163.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

static void cc1101_irq(FAR struct cc1101_dev_s *dev, bool enable)
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

static void cc1101_pwr(FAR struct cc1101_dev_s *dev, bool enable)
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
  FAR struct spi_dev_s *spi    = NULL;
  FAR struct cc1101_dev_s *dev = NULL;

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
