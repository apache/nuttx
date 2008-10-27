/****************************************************************************
 * examples/usbstorage/usbstrg_main.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/usbdev.h>
#include <nuttx/usbdev_trace.h>

#include "usbstrg.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * user_initialize
 ****************************************************************************/

#ifndef CONFIG_HAVE_WEAKFUNCTIONS
void user_initialize(void)
{
  /* Stub that must be provided only if the toolchain does not support weak
   * functions.
   */
}
#endif

/****************************************************************************
 * user_start
 ****************************************************************************/

int user_start(int argc, char *argv[])
{
  void *handle;
  int ret;

  /* Initialize USB trace output IDs */

  usbtrace_enable(
    TRACE_INIT_BIT|            /* Initialization events */
    TRACE_EP_BIT|              /* Endpoint API calls */
    TRACE_DEV_BIT|             /* USB device API calls */
    TRACE_CLASS_BIT|           /* USB class driver API calls */
    TRACE_CLASSAPI_BIT|        /* Other class driver system API calls */
    TRACE_CLASSSTATE_BIT|      /* Track class driver state changes */
    TRACE_OUTREQQUEUED_BIT|    /* Request queued for OUT endpoint */
    TRACE_INREQQUEUED_BIT|     /* Request queued for IN endpoint */
    TRACE_READ_BIT|            /* Read (OUT) action */
    TRACE_WRITE_BIT|           /* Write (IN) action */
    TRACE_COMPLETE_BIT|        /* Request completed */
    TRACE_DEVERROR_BIT|        /* USB controller driver error event */
    TRACE_CLSERROR_BIT         /* USB class driver error event */
  );

  /* Register block drivers (architecture-specific) */

  message("user_start: Creating block drivers\n");
  ret = usbstrg_archinitialize();
  if (ret < 0)
    {
      message("user_start: usbstrg_archinitialize failed: %d\n", -ret);
      return 1;
    }

  /* Then exports the LUN(s) */

  message("user_start: Configuring with NLUNS=%d\n", CONFIG_EXAMPLES_USBSTRG_NLUNS);
  ret = usbstrg_configure(CONFIG_EXAMPLES_USBSTRG_NLUNS, &handle);
  if (ret < 0)
    {
      message("user_start: usbstrg_configure failed: %d\n", -ret);
      usbstrg_uninitialize(handle);
      return 2;
    }
  message("user_start: handle=%p\n", handle);

  message("user_start: Bind LUN=0 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH1);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH1, 0, 0, 0, FALSE);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 1 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH1, -ret);
      usbstrg_uninitialize(handle);
      return 2;
    }

#if CONFIG_EXAMPLES_USBSTRG_NLUNS > 1

  message("user_start: Bind LUN=1 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH2);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH2, 1, 0, 0, FALSE);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 2 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH2, -ret);
      usbstrg_uninitialize(handle);
      return 3;
    }

#if CONFIG_EXAMPLES_USBSTRG_NLUNS > 2

  message("user_start: Bind LUN=2 to %s\n", CONFIG_EXAMPLES_USBSTRG_DEVPATH3);
  ret = usbstrg_bindlun(handle, CONFIG_EXAMPLES_USBSTRG_DEVPATH3, 2, 0, 0, FALSE);
  if (ret < 0)
    {
      message("user_start: usbstrg_bindlun failed for LUN 3 using %s: %d\n",
               CONFIG_EXAMPLES_USBSTRG_DEVPATH3, -ret);
      usbstrg_uninitialize(handle);
      return 4;
    }

#endif
#endif

  ret = usbstrg_exportluns(handle);
  if (ret < 0)
    {
      message("user_start: usbstrg_exportluns failed: %d\n", -ret);
      usbstrg_uninitialize(handle);
      return 5;
    }

  /* Now just hang around and monitor the USB storage activity */

#ifndef CONFIG_DISABLE_SIGNALS
  for (;;)
    {
      sleep(5);
      message("user_start: Still alive\n");
    }
#else
     message("user_start: Exiting\n");
 #endif
}

