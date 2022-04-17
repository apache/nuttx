/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_usbdevserialstr.c
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

#include <stdio.h>

#include "cxd56_uid.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR

static char g_serialstr[CONFIG_BOARDCTL_UNIQUEID_SIZE * 2 + 1];

const char *board_usbdev_serialstr(void)
{
  uint8_t uid[CONFIG_BOARDCTL_UNIQUEID_SIZE];

  cxd56_get_uniqueid(uid);

  snprintf(g_serialstr, sizeof(g_serialstr),
           "%02X%02X%02X%02X%02X",
           uid[0], uid[1], uid[2], uid[3], uid[4]);

  return g_serialstr;
}

#endif
