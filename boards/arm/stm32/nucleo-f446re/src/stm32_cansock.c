/****************************************************************************
 * boards/arm/stm32/nucleo-f446re/src/stm32_cansock.c
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

#include "stm32_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if !defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN2)
#  error "No CAN is enable. Please eneable at least one CAN device"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_cansock_setup
 *
 * Description:
 *  Initialize CAN socket interface
 *
 ****************************************************************************/

int stm32_cansock_setup(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_STM32_CAN1
  /* Call stm32_caninitialize() to get an instance of the CAN interface */

  ret = stm32_cansockinitialize(1);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get CAN interface %d\n", ret);
      goto errout;
    }
#endif

#ifdef CONFIG_STM32_CAN2
  /* Call stm32_caninitialize() to get an instance of the CAN interface */

  ret = stm32_cansockinitialize(2);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get CAN interface %d\n", ret);
      goto errout;
    }
#endif

errout:
  return ret;
}
