/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_gauge.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <debug.h>

#include <stdio.h>
#include <errno.h>

#include "cxd56_gauge.h"

#if defined(CONFIG_CXD56_GAUGE)

static int g_gaugeinitialized = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gauge_initialize
 *
 * Description:
 *   Initialize and register battery gauge driver
 *
 ****************************************************************************/

int board_gauge_initialize(FAR const char *devpath, FAR int16_t *gaugemeter)
{
  int ret;

  if (!g_gaugeinitialized)
    {
      ret = cxd56_gauge_initialize(devpath);
      if (ret < 0)
        {
          _err("ERROR: Failed to initialize gauge.\n");
          return -ENODEV;
        }

      g_gaugeinitialized = 1;
    }

  return OK;
}

/****************************************************************************
 * Name: board_gauge_uninitialize
 *
 * Description:
 *   Uninitialize and unregister battery gauge driver
 *
 ****************************************************************************/

int board_gauge_uninitialize(FAR const char *devpath)
{
  int ret;

  if (g_gaugeinitialized)
    {
      ret = cxd56_gauge_uninitialize(devpath);
      if (ret)
        {
          _err("ERROR: Failed to finalize gauge.\n");
          return -ENODEV;
        }

      g_gaugeinitialized = 0;
    }

  return OK;
}

#endif
