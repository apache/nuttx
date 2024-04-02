/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_twai.c
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

#include "espressif/esp_twai.h"

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_twai_setup
 *
 * Description:
 *  Initialize TWAI and register the TWAI device
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple TWAI interfaces)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int board_twai_setup(int port)
{
#ifdef CONFIG_ESPRESSIF_TWAI
  struct can_dev_s *twai;
  int ret;

  /* Call esp_twaiinitialize() to get an instance of the TWAI
   * interface
   * */

  twai = esp_twaiinitialize(port);
  if (twai == NULL)
    {
      canerr("ERROR:  Failed to get TWAI interface\n");
      return -ENODEV;
    }

#ifdef CONFIG_ESPRESSIF_TWAI0
  /* Register the TWAI driver at "/dev/can0" */

  ret = can_register("/dev/can0", twai);
  if (ret < 0)
    {
      canerr("ERROR: TWAI0 register failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_ESPRESSIF_TWAI0 */

#ifdef CONFIG_ESPRESSIF_TWAI1
  /* Register the TWAI driver at "/dev/can1" */

  ret = can_register("/dev/can1", twai);
  if (ret < 0)
    {
      canerr("ERROR: TWAI1 register failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_ESPRESSIF_TWAI1 */

  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_CAN */
