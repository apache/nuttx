/****************************************************************************
 * boards/xtensa/esp32/ttgo_eink5_v2/src/esp32_twai.c
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

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
/* #include "arm_arch.h" */

#include "esp32_twai.h"
#include "ttgo_eink5_v2.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define TWAI_PORT0 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_twai_setup
 *
 * Description:
 *  Initialize TWAI and register the TWAI device
 *
 ****************************************************************************/

int esp32_twai_setup(void)
{
#ifdef CONFIG_ESP32_TWAI0
  struct can_dev_s *twai;
  int ret;

  /* Call esp32_twaiinitialize() to get an instance of the TWAI0
   * interface
   * */

  twai = esp32_twaiinitialize(TWAI_PORT0);
  if (twai == NULL)
    {
      canerr("ERROR:  Failed to get TWAI0 interface\n");
      return -ENODEV;
    }

  /* Register the TWAI0 driver at "/dev/can0" */

  ret = can_register("/dev/can0", twai);
  if (ret < 0)
    {
      canerr("ERROR: TWAI1 register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
