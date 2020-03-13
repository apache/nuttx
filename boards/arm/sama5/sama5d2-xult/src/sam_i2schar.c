/****************************************************************************
 *  boards/arm/sama5/sama5d2-xult/src/sam_i2schar.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/audio/i2s.h>

#include "sam_ssc.h"
#include "sama5d2-xult.h"

#if defined(CONFIG_AUDIO_I2SCHAR) && \
   (defined(CONFIG_SAMA5_SSC0) || defined(CONFIG_SAMA5_SSC1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMA5D3XPLAINED_SSC_PORT
#  if defined(CONFIG_SAMA5_SSC0)
#    define CONFIG_SAMA5D3XPLAINED_SSC_PORT 0
#  elif defined(CONFIG_SAMA5_SSC1)
#    define CONFIG_SAMA5D3XPLAINED_SSC_PORT 1
#  endif
#endif

#ifndef CONFIG_SAMA5D3XPLAINED_I2SCHAR_MINOR
#  define CONFIG_SAMA5D3XPLAINED_I2SCHAR_MINOR 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2schar_devinit
 *
 * Description:
 *   All architectures must provide the following interface in order to work
 *   with apps/examples/i2schar.
 *
 ****************************************************************************/

int i2schar_devinit(void)
{
  static bool initialized = false;
  struct i2s_dev_s *i2s;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call sam_ssc_initialize() to get an instance of the SSC/I2S
       * interface
       */

      i2s = sam_ssc_initialize(CONFIG_SAMA5D3XPLAINED_SSC_PORT);
      if (!i2s)
        {
          _err("ERROR: Failed to get the SAMA5 SSC/I2S driver for SSC%d\n",
              CONFIG_SAMA5D3XPLAINED_SSC_PORT);
          return -ENODEV;
        }

      /* Register the I2S character driver at "/dev/i2schar0" */

      ret = i2schar_register(i2s, CONFIG_SAMA5D3XPLAINED_I2SCHAR_MINOR);
      if (ret < 0)
        {
          aerr("ERROR: i2schar_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_AUDIO_I2SCHAR && (CONFIG_SAMA5_SSC0 || CONFIG_SAMA5_SSC1) */
