/*****************************************************************************
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
 ****************************************************************************/

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <syslog.h>

#include "stm32_butterfly2.h"

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   Initializes low level pins for the drivers.
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  stm32_led_initialize();
  stm32_spidev_initialize();
  stm32_usb_initialize();
}

/*****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Initializes upper half drivers with board specific settings
 *
 * Returned value:
 *   0 on sucess or errno value of failed init function.
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
  int rv = 0;

#ifdef CONFIG_MMCSD
  if ((rv = stm32_mmcsd_initialize(CONFIG_NSH_MMCSDMINOR)) < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n");
      return rv;
    }
#endif

#ifdef CONFIG_USBHOST
  if ((rv = stm32_usbhost_initialize()) < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USB host: %d\n", rv);
      return rv;
    }
#endif

  return rv;
}
