/*******************************************************************************
 * configs/stm32butterfly2/src/boot.c
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 ******************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <syslog.h>

#include "stm32_gpio.h"
#include "stm32_butterfly2.h"

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void stm32_boardinitialize(void)
{
  stm32_led_initialize();
  stm32_spidev_initialize();
  stm32_usb_initialize();
}

int board_app_initialize(uintptr_t arg)
{
  int rv;
  if ((rv = stm32_sdinitialize(CONFIG_NSH_MMCSDMINOR)) < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n");
      return rv;
    }

  if ((rv = stm32_usbhost_initialize()) < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", rv);
      return rv;
    }

  return 0;
}
