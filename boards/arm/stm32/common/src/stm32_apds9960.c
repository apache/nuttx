/****************************************************************************
 * boards/arm/stm32/common/src/stm32_apds9960.c
 *
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
#include <stdio.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/apds9960.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct stm32_apds9960config_s
{
  /* Configuration structure as seen by the APDS-9960 driver */

  struct apds9960_config_s config;

  /* Additional private definitions only known to this driver */

  FAR void *arg;  /* Argument to pass to the interrupt handler */
  FAR xcpt_t isr; /* ISR Handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  apds9960_irq_attach(FAR struct apds9960_config_s *state,
                                xcpt_t isr, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the APDS-9960
 * driver.  This structure provides information about the configuration
 * of the APDS-9960 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_apds9960config_s g_apds9960config =
{
  .config =
    {
      .irq_attach  = apds9960_irq_attach,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the APDS-9960 interrupt handler to the GPIO interrupt */

static int apds9960_irq_attach(FAR struct apds9960_config_s *state,
                               xcpt_t isr, FAR void *arg)
{
  irqstate_t flags;

  sninfo("apds9960_irq_attach\n");

  flags = enter_critical_section();

  /* Setup interrupt for Falling Edge */

  stm32_gpiosetevent(BOARD_APDS9960_GPIO_INT, false, true, true, isr, arg);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_apds9960_initialize
 *
 * Description:
 *   Initialize and register the APDS9960 gesture sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/gestN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_apds9960_initialize(int devno, int busno)
{
  FAR struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  sninfo("Initializing APDS9960!\n");

  /* Configure the GPIO interrupt */

  stm32_configgpio(BOARD_APDS9960_GPIO_INT);

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Save this i2c in the config */

  g_apds9960config.config.i2c_dev  = i2c;
  g_apds9960config.config.i2c_addr = APDS9960_I2C_ADDR;

  /* Then register the gesture sensor */

  snprintf(devpath, 12, "/dev/gest%d", devno);
  ret = apds9960_register(devpath, &g_apds9960config.config);
  if (ret < 0)
    {
      snerr("ERROR: Failed registering APDS-9960!\n");
    }

  return ret;
}
