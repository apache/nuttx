/****************************************************************************
 * configs/beaglebone-black/src/am335x_lcd.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/lcd/tda19988.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/edid.h>

#include "am335x_lcd.h"
#include "beaglebone-black.h"

#ifdef HAVE_LCD

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int am335x_attach(const struct tda19988_lower_s *lower,
                         xcpt_t handler, void *arg);
static int am335x_enable(const struct tda19988_lower_s *lower, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const strurct tda19988_lower_s g_lower;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_attach
 *
 * Description:
 *   Attach or detach the TDA19988 interrupt handler
 *
 ****************************************************************************/

static int am335x_attach(const struct tda19988_lower_s *lower,
                         xcpt_t handler, void *arg)
{
}

/****************************************************************************
 * Name: am335x_enable
 *
 * Description:
 *   Enable or disable the TDA19988 interrupt
 *
 ****************************************************************************/

static int am335x_enable(const struct tda19988_lower_s *lower, bool enable)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the LCD.  This involves:
 *
 *   1. Initializing the TDA19988 HDMI controller driver
 *   2. Reading EDID data from the connected monitor
 *   3. Selecting a compatible video mode from the EDID data
 *   4. Initializing the LCD controller using this video mode
 *   5. Initializing the HDMI controller using the video mode
 *
 ****************************************************************************/

void up_fbinitialize(int display)
{
  /* Initialize the TDA19988 GPIO interrupt input */
  /* Initialize the TDA19988 lower half state instance */
  /* Initialize the TDA19988 HDMI controller driver */
  /* Allocate a buffer to hold the EDID data */
  /* Read raw EDID data from the connected monitor */
  /* Select a compatible video mode from the EDID data */
  /* Free the allocated EDID buffer */
  /* Convert the video mode to a AM335X LCD panel configuration */
  /* Initialize the LCD controller using this video mode */
  /* Convert the EDID video mode to a TDA19988 video mode */
  /* Initialize the HDMI controller using the TDA19988 video mode */
}

#endif /* HAVE_LCD */
