/************************************************************************************
 * configs/sama5d3x-ek/src/sam_i2schar.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/audio/i2s.h>

#include "sam_ssc.h"
#include "sama5d3x-ek.h"

#if defined(CONFIG_AUDIO_I2SCHAR) && \
   (defined(CONFIG_SAMA5_SSC0) || defined(CONFIG_SAMA5_SSC1))

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_SAMA5D3xEK_SSC_PORT
#  if defined(CONFIG_SAMA5_SSC0)
#    define CONFIG_SAMA5D3xEK_SSC_PORT 0
#  elif defined(CONFIG_SAMA5_SSC1)
#    define CONFIG_SAMA5D3xEK_SSC_PORT 1
#  endif
#endif

#ifndef CONFIG_SAMA5D3xEK_I2SCHAR_MINOR
#  define CONFIG_SAMA5D3xEK_I2SCHAR_MINOR 0
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: i2schar_devinit
 *
 * Description:
 *   All architectures must provide the following interface in order to work with
 *   apps/examples/i2schar.
 *
 ************************************************************************************/

int i2schar_devinit(void)
{
  static bool initialized = false;
  struct i2s_dev_s *i2s;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call sam_ssc_initialize() to get an instance of the SSC/I2S interface */

      i2s = sam_ssc_initialize(CONFIG_SAMA5D3xEK_SSC_PORT);
      if (!i2s)
        {
          dbg("ERROR: Failed to get the SAMA5 SSC/I2S driver for SSC%d\n",
              CONFIG_SAMA5D3xEK_SSC_PORT);
          return -ENODEV;
        }

      /* Register the I2S character driver at "/dev/i2schar0" */

      ret = i2schar_register(i2s, CONFIG_SAMA5D3xEK_I2SCHAR_MINOR);
      if (ret < 0)
        {
          adbg("ERROR: i2schar_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_AUDIO_I2SCHAR && (CONFIG_SAMA5_SSC0 || CONFIG_SAMA5_SSC1) */
