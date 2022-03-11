/****************************************************************************
 * arch/arm/src/am335x/am335x_sysclk.c
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

#include <stdint.h>
#include <errno.h>

#include "arm_internal.h"
#include "hardware/am335x_scm.h"
#include "am335x_sysclk.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_get_sysclk
 *
 * Description:
 *   Return the sysclk frequency
 *
 ****************************************************************************/

int32_t am335x_get_sysclk(void)
{
  uint32_t regval;

  /* Read the input clock freq from the control module. */

  regval = getreg32(AM335X_SCM_CTRL_STATUS);

  /* Return the frequency of the configured crystal */

  switch (regval & SCM_CTRL_STATUS_SYSBOOT1_MASK)
    {
      case SCM_CTRL_STATUS_SYSBOOT1_19p2MHZ: /* 19.2Mhz */
        return 19200000;

      case SCM_CTRL_STATUS_SYSBOOT1_24MHZ: /* 24Mhz */
        return 24000000;

      case SCM_CTRL_STATUS_SYSBOOT1_25MHZ: /* 25Mhz */
        return 25000000;

      case SCM_CTRL_STATUS_SYSBOOT1_26MHZ: /* 26Mhz */
        return 26000000;
    }

  return -EINVAL;  /* Should never happen */
}
