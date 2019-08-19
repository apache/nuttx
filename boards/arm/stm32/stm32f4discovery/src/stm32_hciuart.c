/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_xen1210.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author:  Alan Carvalho de Assis <acassis@gmail.com>
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/wireless/bluetooth/bt_uart.h>

#include "stm32_hciuart.h"
#include "stm32f4discovery.h"

#include <arch/board/board.h>

#ifdef HAVE_HCIUART

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hciuart_dev_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   Bluetooth HCI UART driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int hciuart_dev_initialize(void)
{
  const struct btuart_lowerhalf_s *lower;
  int ret;

  /* Perform one-time initialization */

  hciuart_initialize();

  /* Instantiate the HCI UART lower half interface */

  lower = hciuart_instantiate(HCIUART_SERDEV);
  if (lower == NULL)
    {
      wlerr("ERROR: Failed to instantiate HCIUART%d\n", HCIUART_SERDEV + 1);
      return -ENODEV;
    }

  /* Then initialize the HCI UART upper half driver with the bluetooth stack */

  ret = btuart_register(lower);
  if (ret < 0)
    {
      wlerr("ERROR: btuart_register() failed: %d\n", ret);
    }

  return ret;
}

#endif /* HAVE_HCIUART */
