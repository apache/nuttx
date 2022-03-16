/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_reset.c
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

#include "arm_internal.h"
#include "lpc54_reset.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_reset
 *
 * Description:
 *   Reset the selected peripheral
 *
 ****************************************************************************/

void lpc54_reset(uintptr_t setreg, uintptr_t clrreg,
                 uintptr_t statreg, uint32_t mask)
{
  /* Set the bit to put the peripheral in reset */

  putreg32(mask, setreg);

  /* Wait until the peripheral is in reset */

  while ((getreg32(statreg) & mask) == 0)
    {
    }

  /* Clear the bit to take the peripheral out of reset */

  putreg32(mask, clrreg);

  /* Wait until the peripheral is out of reset */

  while ((getreg32(statreg) & mask) != 0)
    {
    }
}
