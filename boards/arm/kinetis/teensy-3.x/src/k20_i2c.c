/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/k20_i2c.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author:  Matias v01d <phreakuencies@gmail.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "kinetis.h"
#include "kinetis_i2c.h"
#include "teensy-3x.h"

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

void kinetis_i2cdev_initialize(void)
{
  i2c_dev = NULL;

#if defined(CONFIG_KINETIS_I2C0)
  i2c_dev = kinetis_i2cbus_initialize(0);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 0);
#endif
#endif

#if defined(CONFIG_KINETIS_I2C1)
  i2c_dev  = kinetis_i2cbus_initialize(1);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 1);
#endif
#endif
}

#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 */
