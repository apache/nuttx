/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_mcan.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "sam_mcan.h"
#include "jti-toucan2.h"
#include "sam_pio.h"

#if defined (CONFIG_SAMA5_MCAN0) || defined (CONFIG_SAMA5_MCAN1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void board_can_pio_control_initialize(void)
{
   sam_configpio(PIO_MCAN0_SILENT_MODE);
   sam_configpio(PIO_MCAN1_SILENT_MODE);
   sam_configpio(PIO_MCAN0_TERMINATION_ENABLE);
   sam_configpio(PIO_MCAN1_TERMINATION_ENABLE);

  /* ENABLE CAN TRANSCEIVERS */

   sam_piowrite(PIO_MCAN0_SILENT_MODE, false);
   sam_piowrite(PIO_MCAN1_SILENT_MODE, false);


}
/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device(s)
 *
 ****************************************************************************/ 

int sam_can_setup(void)
{
#if defined(CONFIG_SAMA5_MCAN0) || defined(CONFIG_SAMA5_MCAN1)
  struct can_dev_s *can0;
  int ret;
# ifndef CONFIG_SAMA5_MCAN0
  can0 = sam_mcan_initialize(MCAN1);
  if (can0 == NULL)
    {
      canerr("ERROR:  Failed to get interface mcan1\n");
      return -ENODEV;
    }
  ret = can_register("/dev/can0", can0);
  if (ret < 0)
    {
      canerr("ERROR: can_register can0 failed: %d\n", ret);
      return ret;
    }     
# else
  can0 = sam_mcan_initialize(MCAN0);
  if (can0 == NULL)
    {
      canerr("ERROR:  Failed to get interface mcan0\n");
      return -ENODEV;
    }
  ret = can_register("/dev/can0", can0);
  if (ret < 0)
    {
      canerr("ERROR: can_register can0 failed: %d\n", ret);
      return ret;
    }  
# if defined(CONFIG_SAMA5_MCAN1)
  struct can_dev_s *can1;
  can1 = sam_mcan_initialize(MCAN1);
  if (can1 == NULL)
    {
      canerr("ERROR:  Failed to get interface mcan1\n");
      return -ENODEV;
    }
  ret = can_register("/dev/can1", can1);

  if (ret < 0)
    {
      canerr("ERROR: can_register can1 failed: %d\n", ret);
      return ret;
    }
# endif
# endif

  return OK;

#else
  return -ENODEV;
#endif

}

#endif /* CONFIG_SAMA5_MCAN */
