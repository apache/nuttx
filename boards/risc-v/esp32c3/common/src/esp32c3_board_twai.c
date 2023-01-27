/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp32c3_board_twai.c
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

#include "esp32c3_twai.h"

#include "esp32c3_board_twai.h"

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
 * Name: board_twai_setup
 *
 * Description:
 *   Configure the TWAI driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_twai_setup(void)
{
#ifdef CONFIG_ESP32C3_TWAI0
  struct can_dev_s *twai;
  int ret;

  /* Call esp32c3_twaiinitialize() to get an instance of the TWAI0
   * interface
   * */

  twai = esp32c3_twaiinitialize(TWAI_PORT0);
  if (twai == NULL)
    {
      canerr("ERROR:  Failed to get TWAI0 interface\n");
      return -ENODEV;
    }

  /* Register the TWAI0 driver at "/dev/can0" */

  ret = can_register("/dev/can0", twai);
  if (ret < 0)
    {
      canerr("ERROR: TWAI0 register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
