/****************************************************************************
 * graphics/nxmu/nx_start.c
 *
 *   Copyright (C) 2013, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#include <nuttx/nx/nx.h>

#include "nxfe.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_nxserver_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_server
 *
 * Description:
 *   NX server thread.  This is the entry point into the server kernel
 *   thread that serializes the multi-threaded accesses to the display.
 *
 * Input Parameters:
 *   Standard task start-up parameters (none of which are used)
 *
 * Returned Value:
 *   This function does not normally return but may exit with EXIT_FAILURE
 *   under certain error conditions.
 *
 ****************************************************************************/

static int nx_server(int argc, char *argv[])
{
  FAR NX_DRIVERTYPE *dev;
  int ret;

#if defined(CONFIG_NXSTART_EXTERNINIT)
  /* Use external graphics driver initialization */

  dev = board_graphics_setup(CONFIG_NXSTART_DEVNO);
  if (!dev)
    {
      gerr("ERROR: board_graphics_setup failed, devno=%d\n", CONFIG_NXSTART_DEVNO);
      return EXIT_FAILURE;
    }

#elif defined(CONFIG_NX_LCDDRIVER)
  /* Initialize the LCD device */

  ret = board_lcd_initialize();
  if (ret < 0)
    {
      gerr("ERROR: board_lcd_initialize failed: %d\n", ret);
      return EXIT_FAILURE;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_NXSTART_DEVNO);
  if (!dev)
    {
      gerr("ERROR: board_lcd_getdev failed, devno=%d\n",
           CONFIG_NXSTART_DEVNO);
      return EXIT_FAILURE;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

#else /* CONFIG_NX_LCDDRIVER */
  /* Initialize the frame buffer device.
   * REVISIT: display == 0 is assumed.
   */

  ret = up_fbinitialize(CONFIG_NXSTART_DISPLAYNO);
  if (ret < 0)
    {
      gerr("ERROR: up_fbinitialize failed: %d\n", ret);
      return EXIT_FAILURE;
    }

  dev = up_fbgetvplane(CONFIG_NXSTART_DISPLAYNO, CONFIG_NXSTART_VPLANE);
  if (!dev)
    {
      gerr("ERROR: up_fbgetvplane failed, vplane=%d\n", CONFIG_NXSTART_VPLANE);
      return EXIT_FAILURE;
    }

#endif /* CONFIG_NX_LCDDRIVER */

  /* Then start the server (nx_run does not normally return) */

  ret = nx_run(dev);
  ginfo("nx_run returned: %d\n", ret);
  UNUSED(ret);

  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_start
 *
 * Description:
 *   nx_start() provides a wrapper function to simplify and standardize the
 *   starting of the NX server.
 *
 *   nx_start() can be called (indirectly) from applications via the
 *   boardctl() interface with the BOARDIOC_NX_START command.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  This indicates that the NX server
 *   has been successfully started, is running, and waiting to accept
 *   connections from NX clients.
 *
 *   A negated errno value is returned on failure.  The errno value indicates
 *   the nature of the failure.
 *
 ****************************************************************************/

int nx_start(void)
{
  /* Do nothing is the server has already been started */

  if (!g_nxserver_started)
    {
      pid_t server;

      /* Start the server kernel thread */

      ginfo("Starting server task\n");
      server = kthread_create("NX Server", CONFIG_NXSTART_SERVERPRIO,
                              CONFIG_NXSTART_SERVERSTACK, nx_server, NULL);
      if (server < 0)
        {
          gerr("ERROR: Failed to create nx_server kernel thread: %d\n",
               (int)server);
          return (int)server;
        }

      g_nxserver_started = true;

      /* Wait a bit to make sure that the server get started.  NOTE that
       * this operation cannot be done from the IDLE thread!
       */

      nxsig_usleep(50*1000);
    }

  return OK;
}
