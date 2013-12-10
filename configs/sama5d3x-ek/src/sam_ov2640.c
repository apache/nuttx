/****************************************************************************
 * configs/sama5d3x-ek/src/sam_ov2640.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <debug.h>

#include <nuttx/i2c.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/ov2640.h>

#include "up_arch.h"

#include "sam_periphclks.h"
#include "sam_lcd.h"
#include "sam_pck.h"
#include "sama5d3x-ek.h"

#ifdef HAVE_CAMERA

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ov2640_lcd_initialize
 ****************************************************************************/

static inline FAR struct fb_vtable_s *ov2640_lcd_initialize(void)
{
  FAR struct fb_vtable_s *vplane;
  int ret;

  /* Initialize the frame buffer device */

  ret = up_fbinitialize();
  if (ret < 0)
    {
      gdbg("ERROR: up_fbinitialize failed: %d\n", -ret);
      return NULL;
    }

  vplane = up_fbgetvplane(0);
  if (!vplane)
    {
      gdbg("ERROR: up_fbgetvplane failed\n");
    }

  return vplane;
}

/****************************************************************************
 * Name: ov2640_camera_initialize
 ****************************************************************************/

static inline int ov2640_camera_initialize(void)
{
  FAR struct i2c_dev_s *i2c;
  int ret;

  /* Get the I2C driver that interfaces with the camers (OV2640_BUS)*/

  i2c = up_i2cinitialize(OV2640_BUS);
  if (!i2c)
    {
      fdbg("ERROR: Failed to initialize TWI%d\n", OV2640_BUS);
      return EXIT_FAILURE;
    }

  /* Enable clocking to the ISI peripheral */

  sam_isi_enableclk();

#warning Missing Logic

  /* Initialize the OV2640 camera */

  ret = ov2640_initialize(i2c);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to initialize the OV2640: %d\n", ret);
      return EXIT_FAILURE;
    }

  return EXIT_FAILURE;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: ov2640_main
 *
 * Description:
 *   Entry point for the OV2640 Camera Demo
 *
 ************************************************************************************/

int ov2640_main(int argc, char *argv[])
{
  FAR struct fb_vtable_s *vplane;
  int ret;

  /* First, initialize the display */

  vplane = ov2640_lcd_initialize();
  if (!vplane)
    {
      gdbg("ERROR: ov2640_lcd_initialize failed\n");
      return  EXIT_FAILURE;
    }

  /* Then, initialize the camera */

  ret = ov2640_camera_initialize();
  if (ret != EXIT_SUCCESS)
    {
      gdbg("ERROR: ov2640_camera_initialize failed\n");
      return  EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}

#endif /* HAVE_CAMERA */
