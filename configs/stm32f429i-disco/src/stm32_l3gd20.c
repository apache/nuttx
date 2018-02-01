/****************************************************************************
 * configs/stm32f429i-disco/src/stm32_l3gd20.c
 *
 *   Copyright (C) 2017 Gregory Nutt.  All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/l3gd20.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32f429i-disco.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPI) & defined(CONFIG_SENSORS_L3GD20)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int l3gd20_attach(FAR struct l3gd20_config_s * cfg, xcpt_t irq);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Only one L3GD20 device on board */

static struct l3gd20_config_s g_l3gd20_config =
{
  .attach = l3gd20_attach,
  .irq = L3GD20_IRQ,
  .spi_devid = SPIDEV_ACCELEROMETER(0)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: l3gd20_attach()
 *
 * Description: Attach the l3gd20 interrupt handler to the GPIO interrupt
 *
 ****************************************************************************/

static int l3gd20_attach(FAR struct l3gd20_config_s *cfg, xcpt_t irq)
{
  return stm32_gpiosetevent(GPIO_L3GD20_DREADY, true, false, true, irq, NULL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_l3gd20initialize()
 *
 * Description:
 *   Initialize and register the L3GD20 3 axis gyroscope sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyro0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_l3gd20initialize(FAR const char *devpath)
{
  int ret = 0;
  struct spi_dev_s *spi;

  /* Configure DREADY IRQ input */

  stm32_configgpio(GPIO_L3GD20_DREADY);

  /* Initialize SPI */

  spi = stm32_spi5initialize();

  if (!spi)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Then register the gyro */

  ret = l3gd20_register(devpath, spi, &g_l3gd20_config);
  if (ret != OK)
    {
      goto errout;
    }

errout:
  return ret;
}

#endif  /* CONFIG_SPI && CONFIG_SENSORS_L3GD20 */
