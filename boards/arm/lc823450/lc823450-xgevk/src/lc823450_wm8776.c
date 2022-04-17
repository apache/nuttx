/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_wm8776.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/wm8776.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "lc823450_i2c.h"
#include "lc823450_i2s.h"
#include "lc823450-xgevk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WM8776_I2C_PORTNO 0   /* On I2C0 */
#define WM8776_I2C_ADDR   0x1a

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wm8776_lower_s g_wm8776info =
{
  .address = WM8776_I2C_ADDR,
  .frequency = 400000,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_wm8776initialize
 ****************************************************************************/

int lc823450_wm8776initialize(int minor)
{
  struct audio_lowerhalf_s *wm8776;
  struct audio_lowerhalf_s *pcm;
  struct i2c_master_s *i2c;
  struct i2s_dev_s *i2s;
  char devname[12];
  int ret;

  ainfo("Initializing WM8776\n");

  /* Initialize I2C */

  i2c = lc823450_i2cbus_initialize(WM8776_I2C_PORTNO);

  if (!i2c)
    {
      return -ENODEV;
    }

  i2s = lc823450_i2sdev_initialize();

#ifdef CONFIG_AUDIO_I2SCHAR
  i2schar_register(i2s, 0);
#endif

  wm8776 = wm8776_initialize(i2c, i2s, &g_wm8776info);

  if (!wm8776)
    {
      auderr("ERROR: Failed to initialize the WM8904\n");
      return -ENODEV;
    }

  pcm = pcm_decode_initialize(wm8776);

  if (!pcm)
    {
      auderr("ERROR: Failed create the PCM decoder\n");
      return  -ENODEV;
    }

  snprintf(devname, 12, "pcm%d",  minor);

  ret = audio_register(devname, pcm);

  if (ret < 0)
    {
      auderr("ERROR: Failed to register /dev/%s device: %d\n", devname, ret);
    }

  return 0;
}
