/****************************************************************************
 * boards/arm/stm32/odrive36/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <arch/board/board.h>

#include <nuttx/usb/cdcacm.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_SENSORS_QENCODER
#  include "board_qencoder.h"
#  include "stm32_qencoder.h"
#endif

#include "odrive.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
    }
#endif /* CONFIG_FS_PROCFS */

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE)
  /* Initialize CDCACM */

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

#if defined(CONFIG_STM32_TIM3_QE) && defined(CONFIG_SENSORS_QENCODER)
  /* Initialize and register the qencoder driver - TIM3 */

  ret = board_qencoder_initialize(0, 3);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
    }

  /* Connect QE index pin */

  ret = stm32_qe_index_init(3, GPIO_QE3_INDEX);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register qe index pin: %d\n", ret);
    }
#endif

#if defined(CONFIG_STM32_TIM4_QE) && defined(CONFIG_SENSORS_QENCODER)
  /* Initialize and register the qencoder driver - TIM4 */

  ret = board_qencoder_initialize(1, 4);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n", ret);
    }

  /* Connect QE index pin */

  ret = stm32_qe_index_init(4, GPIO_QE4_INDEX);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to register qe index pin: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_FOC
  /* Initialize and register FOC devices */

  ret = stm32_foc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_foc_setup failed: %d\n", ret);
    }
#endif

  return ret;
}
