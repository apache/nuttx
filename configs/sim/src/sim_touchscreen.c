/****************************************************************************
 * config/sim/src/sim_touchscreen.c
 *
 *   Copyright (C) 2011, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/video/fb.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#ifdef CONFIG_VNCSERVER
#  include <nuttx/video/vnc.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Pick a background color */

#ifndef CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR
#  define CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR 0x007b68ee
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_touchscreen_s
{
  NXHANDLE hnx;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sim_touchscreen_s g_simtc;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_tsc_setup()
 *
 * Description:
 *   Perform architecuture-specific initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

int board_tsc_setup(int minor)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the simulated frame buffer device.  We need to create an
   * X11 window to support the mouse-driven touchscreen simulation.
   */

  iinfo("Initializing framebuffer\n");
  ret = up_fbinitialize(0);
  if (ret < 0)
    {
      ierr("ERROR: up_fbinitialize failed: %d\n", -ret);
      goto errout;
    }

  dev = up_fbgetvplane(0, 0);
  if (!dev)
    {
      ierr("ERROR: up_fbgetvplane 0 failed\n");
      ret = -ENODEV;
      goto errout_with_fb;
    }

  /* Then open NX */

  iinfo("Open NX\n");
  g_simtc.hnx = nx_open(dev);
  if (!g_simtc.hnx)
    {
      ret = -errno;
      ierr("ERROR: nx_open failed: %d\n", ret);
      goto errout_with_fb;
    }

#ifdef CONFIG_VNCSERVER
  /* Setup the VNC server to support keyboard/mouse inputs */

  ret = vnc_default_fbinitialize(0, g_simtc.hnx);
  if (ret < 0)
    {
      ierr("ERROR: vnc_default_fbinitialize failed: %d\n", ret);
      goto errout_with_fb;
    }
#endif

  /* Set the background to the configured background color */

  iinfo("Set background color=%d\n", CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR);

  color = CONFIG_EXAMPLES_TOUCHSCREEN_BGCOLOR;
  ret = nx_setbgcolor(g_simtc.hnx, &color);
  if (ret < 0)
    {
      ierr("ERROR: nx_setbgcolor failed: %d\n", ret);
      goto errout_with_nx;
    }

  /* Finally, initialize the touchscreen simulation on the X window */

  ret = sim_tsc_initialize(minor);
  if (ret < 0)
    {
      ierr("ERROR: sim_tsc_initialize failed: %d\n", ret);
      goto errout_with_nx;
    }

  return OK;

errout_with_nx:
  nx_close(g_simtc.hnx);
  goto errout;

errout_with_fb:
  up_fbuninitialize(0);
errout:
  return ret;
}

/****************************************************************************
 * Name: board_tsc_teardown()
 *
 * Description:
 *   Perform architecuture-specific un-initialization of the touchscreen
 *   hardware.  This interface must be provided by all configurations
 *   using apps/examples/touchscreen
 *
 ****************************************************************************/

void board_tsc_teardown(void)
{
  /* Shut down the touchscreen driver */

  sim_tsc_uninitialize();

  /* Close NX */

  nx_close(g_simtc.hnx);
}
