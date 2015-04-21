/************************************************************************************
 * configs/olimex-lpc1766stk/src/lpc17_touchscreen.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/input/touchscreen.h>

#ifdef CONFIG_USBHOST_HIDMOUSE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Both CONFIG_DEBUG_INPUT and CONFIG_DEBUG_USB could apply to this file.
 * We assume here that CONFIG_DEBUG_INPUT might be enabled separately, but
 * CONFIG_DEBUG_USB implies both.
 */

#ifndef CONFIG_DEBUG_INPUT
#  undef  idbg
#  define idbg    udbg
#  undef  illdbg
#  define illdbg  ulldbg
#  undef  ivdbg
#  define ivdbg   uvdbg
#  undef  illvdbg
#  define illvdbg ullvdbg
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tsc_setup
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this
 *   function.  This function is called by application-specific, setup logic
 *   to configure the USB HID mouse driver that emulates a touchscreen
 *   device.  This function will register the driver as /dev/mouseN where N
 *   is the minor device number.
 *
 * Input Parameters:
 *   minor   - The mouse device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_tsc_setup(int minor)
{
  static bool initialized = false;
  int ret;

  idbg("minor %d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Have we already initialized?  Since we never uninitialize we must prevent
   * multiple initializations.  This is necessary, for example, when the
   * touchscreen example is used as a built-in application in NSH and can be
   * called numerous time.  It will attempt to initialize each time.
   */

  if (!initialized)
    {
#ifdef CONFIG_USBHOST_HUB
      /* Initialize USB hub support */

      ret = usbhost_hub_initialize();
      if (ret < 0)
        {
          idbg("ERROR: usbhost_hub_initialize failed: %d\n", ret);
        }
#endif

      /* Initialize and register the USB HID mouse device class */

      ret = usbhost_mouse_init();
      if (ret < 0)
        {
          idbg("Failed to register USB HID mouse device class\n");
          return -ENODEV;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

/****************************************************************************
 * Name: board_tsc_teardown
 *
 * Description:
 *   Each board that supports a touchscreen device must provide this function.
 *   This function is called by application-specific, setup logic to
 *   uninitialize the touchscreen device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_tsc_teardown(void)
{
  /* No support for un-initializing the USB mouse driver.  It will continue
   * to run and process touch interrupts in the background.
   */
}

#endif /* CONFIG_USBHOST_HIDMOUSE */
