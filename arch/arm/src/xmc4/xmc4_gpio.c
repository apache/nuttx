/****************************************************************************
 *  arch/arm/src/xmc4/xmc4_gpio.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <arch/board/board.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/xmc4_ports.h"
#include "xmc4_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_gpio_config
 *
 * Description:
 *   Configure a PIN based on bit-encoded description of the pin,
 *   'pincconfig'.
 *
 ****************************************************************************/

int xmc4_gpio_config(gpioconfig_t pinconfig)
{
#warning Missing logic
  return -EINVAL;
}

/****************************************************************************
 * Name: xmc4_gpio_write
 *
 * Description:
 *   Write one or zero to the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

void xmc4_gpio_write(gpioconfig_t pinconfig, bool value)
{
#warning Missing logic
}

/****************************************************************************
 * Name: xmc4_gpio_read
 *
 * Description:
 *   Read one or zero from the PORT pin selected by 'pinconfig'
 *
 ****************************************************************************/

bool xmc4_gpio_read(gpioconfig_t pinconfig)
{
#warning Missing logic
  return false;
}
