/****************************************************************************
 * configs/lc823450-xgevk/src/lc823450_wm8776.c
 *
 *   Copyright 2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/wm8776.h>

#include <arch/board/board.h>

#include "up_arch.h"
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
 * Name: lc823450_wm8776initialize
 ****************************************************************************/

int lc823450_wm8776initialize(int minor)
{
  FAR struct audio_lowerhalf_s *wm8776;
  FAR struct audio_lowerhalf_s *pcm;
  FAR struct i2c_master_s *i2c;
  FAR struct i2s_dev_s *i2s;
  char devname[12];
  int ret;

  ainfo("Initializing WM8776 \n");

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

