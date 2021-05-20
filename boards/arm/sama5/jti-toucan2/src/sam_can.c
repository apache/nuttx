/****************************************************************************
 * boards/arm/sama5/jti-toucan2/src/sam_can.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
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
#include "arm_arch.h"

#include "sam_can.h"
#include "jti-toucan2.h"
#include <stdio.h>

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

int sam_can_setup(void)
{
#if defined(CONFIG_CAN) 
	
	int ret = OK;
  printf("PRINTF: Setting up CAN\n");
#if defined CONFIG_DEBUG_CAN_INFO
  
  canerr("CANERROR: Setting up CAN\n");
  canwarn("CANWARN: Setting up CAN\n");
  caninfo("CANINFO: Setting up CAN\n");
#else
    printf("no CAN debug enabled\n");
#endif
	#if defined(CONFIG_SAMA5_CAN0)
    struct can_dev_s *can0;
	  /* Call sam_caninitialize() to get an instance of the CAN interface */

	  can0 = sam_caninitialize(0);
	  if (can0 == NULL)
		{
		  canerr("ERROR:  Failed to get CAN 0 interface\n");
		  return -ENODEV;
		}


	  /* Register the CAN driver at "/dev/can0" */

	  ret = can_register("/dev/can0", can0);
	  if (ret < 0)
		{
		  canerr("ERROR: can_register can0 failed: %d\n", ret);
		  return ret;
		}
 
	#endif

	#if defined(CONFIG_SAMA5_CAN1)
    struct can_dev_s *can1;
	  /* Call sam_caninitialize() to get an instance of the CAN interface */

	  can1 = sam_caninitialize(1);
	  if (can1 == NULL)
		{
		  canerr("ERROR:  Failed to get CAN 1 interface\n");
		  return -ENODEV;
		}

	  /* Register the CAN driver at "/dev/can1" */

	  ret = can_register("/dev/can1", can1);
	  if (ret < 0)
		{
		  canerr("ERROR: can_register can1 failed: %d\n", ret);
		  return ret;
		}
	#endif


#endif
  return ret;
}

#endif /* CONFIG_CAN */
