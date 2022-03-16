/****************************************************************************
 * arch/arm/src/sam34/sam4cm_supc.c
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
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/sam_supc.h"

#include "sam4cm_supc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t supc_get_slcd_power_mode(void)
{
  return getreg32(SAM_SUPC_MR) & SUPC_MR_LCDMODE_MASK;
}

void supc_set_slcd_power_mode(uint32_t mode)
{
  uint32_t regval = getreg32(SAM_SUPC_MR);
  regval &= ~SUPC_MR_LCDMODE_MASK;
  regval |= SUPC_MR_KEY | mode;
  putreg32(regval, SAM_SUPC_MR);

  if (mode == SUPC_MR_LCDMODE_LCDOFF)
    {
      while (getreg32(SAM_SUPC_SR) & SUPC_SR_LCDS);
    }
  else
    {
      while (!(getreg32(SAM_SUPC_SR) & SUPC_SR_LCDS));
    }
}

void supc_set_slcd_ldo_output(uint32_t vrout)
{
  uint32_t regval = getreg32(SAM_SUPC_MR);
  regval &= ~SUPC_MR_LCDVROUT_MASK;
  regval |=  SUPC_MR_KEY | vrout;
  putreg32(regval, SAM_SUPC_MR);
}
