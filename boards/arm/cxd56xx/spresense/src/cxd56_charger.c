/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_charger.c
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

#include "cxd56_charger.h"

#if defined(CONFIG_CXD56_CHARGER)

static int g_chargerinitialized = 0;

/****************************************************************************
 * Name: board_charger_initialize
 *
 * Description:
 *   Initialize and register battery charger driver
 *
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_charger_initialize(FAR const char *devpath,
                             FAR int16_t *gaugemeter)
{
  int ret;

  if (!g_chargerinitialized)
    {
      ret = cxd56_charger_initialize(devpath);
      if (ret < 0)
        {
          _err("ERROR: Failed to initialize charger.\n");
          return -ENODEV;
        }

      g_chargerinitialized = 1;
    }

  return OK;
}

/****************************************************************************
 * Name: board_charger_uninitialize
 *
 * Description:
 *   Uninitialize and unregister battery charger driver
 *
 ****************************************************************************/

int board_charger_uninitialize(FAR const char *devpath)
{
  int ret;

  if (g_chargerinitialized)
    {
      ret = cxd56_charger_uninitialize(devpath);
      if (ret)
        {
          _err("ERROR: Failed to finalize charger.\n");
          return -ENODEV;
        }

      g_chargerinitialized = 0;
    }

  return OK;
}

#endif
