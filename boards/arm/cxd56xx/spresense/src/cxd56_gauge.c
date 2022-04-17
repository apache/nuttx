/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_gauge.c
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

int board_gauge_initialize(const char *devpath, int16_t *gaugemeter)
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

int board_gauge_uninitialize(const char *devpath)
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
