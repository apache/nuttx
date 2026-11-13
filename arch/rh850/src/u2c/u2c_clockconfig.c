/****************************************************************************
 * arch/rh850/src/u2c/u2c_clockconfig.c
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

#include <stdint.h>

#include "rh850_internal.h"
#include "hardware/u2c_sysctrl.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: u2c_module_release
 *
 * Description:
 *   Wait for the end of the module reset, then clear its module stop bit.
 *
 ****************************************************************************/

static void u2c_module_release(uintptr_t swmress, uintptr_t msr,
                               uint32_t bit)
{
  while ((getreg32(swmress) & bit) != 0)
    {
    }

  putreg32(U2C_KCPROT_ENABLE, U2C_MSRKCPROT);
  putreg32(getreg32(msr) & ~bit, msr);
  while ((getreg32(msr) & bit) != 0)
    {
    }

  putreg32(U2C_KCPROT_DISABLE, U2C_MSRKCPROT);
  while (getreg32(U2C_MSRKCPROT) != 0)
    {
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_clockconfig
 *
 * Description:
 *   Clock and standby setup of the modules in use.  PLL and CPU clock keep
 *   their reset settings.
 *
 ****************************************************************************/

void rh850_clockconfig(void)
{
#ifdef CONFIG_RH850_RLIN311
  /* CLK_RLIN3 from CLKC_HSB, then RLIN311 out of module stop */

  putreg32(U2C_KCPROT_ENABLE, U2C_CLKKCPROT1);
  putreg32(U2C_CKS_RLIN_HSB, U2C_CKS_RLINC);
  while ((getreg32(U2C_CKS_RLINS) & U2C_CKS_RLIN_MASK) != U2C_CKS_RLIN_HSB)
    {
    }

  putreg32(U2C_KCPROT_DISABLE, U2C_CLKKCPROT1);

  u2c_module_release(U2C_SWMRESS_RLIN3, U2C_MSR_RLIN3, U2C_MSR_RLIN311);
#endif

  /* OSTM0, the system timer */

  u2c_module_release(U2C_SWMRESS_OSTM, U2C_MSR_OSTM, U2C_MSR_OSTM0);
}
