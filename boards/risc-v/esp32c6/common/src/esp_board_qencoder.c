/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_qencoder.c
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

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "espressif/esp_qencoder.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_qencoder_initialize
 *
 * Description:
 *   Initialize the quadrature encoder driver
 *
 ****************************************************************************/

int board_qencoder_initialize(void)
{
  int ret;
  char devpath[12];
  int devno = 0;

  /* Initialize a quadrature encoder interface. */
#ifdef CONFIG_ESP_PCNT_U0_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP_PCNT_U1_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP_PCNT_U2_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ESP_PCNT_U3_QE
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno++);
  ret = esp_qeinitialize(devpath, 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_qeinitialize failed: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}

