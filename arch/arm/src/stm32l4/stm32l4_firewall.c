/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_firewall.c
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

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/stm32l4_syscfg.h"

#include "stm32l4_firewall.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32l4_firewallsetup(struct stm32l4_firewall_t *setup)
{
  uint32_t reg;

  /* Code and nvdata must be aligned to 256 bytes
   * data must be aligned to 64 bytes
   */

  if ((setup->codestart & 0xff) != 0 || (setup->nvdatastart & 0xff) != 0 ||
      (setup->datastart & 0x3f) != 0)
    {
      return -EINVAL;
    }

  /* Code and nvdata length must be a multiple of 256 bytes
   * data length must be a multiple of 64 bytes
   */

  if ((setup->codelen & 0xff) != 0 || (setup->nvdatalen & 0xff) != 0 ||
      (setup->datalen & 0x3f) != 0)
    {
      return -EINVAL;
    }

  /* Code and nvdata must be in flash
   * data must be in SRAM1
   */

  if ((setup->codestart & STM32L4_FLASH_MASK) != STM32L4_FLASH_BASE)
    {
      return -EINVAL;
    }

  if ((setup->nvdatastart & STM32L4_FLASH_MASK) != STM32L4_FLASH_BASE)
    {
      return -EINVAL;
    }

  /* Define address and length registers */

  modifyreg32(STM32L4_FIREWALL_CSSA, FIREWALL_CSSADD_MASK,
              setup->codestart);
  modifyreg32(STM32L4_FIREWALL_CSL, FIREWALL_CSSLENG_MASK,
              setup->codelen);
  modifyreg32(STM32L4_FIREWALL_NVDSSA, FIREWALL_NVDSADD_MASK,
              setup->nvdatastart);
  modifyreg32(STM32L4_FIREWALL_NVDSL, FIREWALL_NVDSLENG_MASK,
              setup->nvdatalen);
  modifyreg32(STM32L4_FIREWALL_VDSSA, FIREWALL_VDSADD_MASK,
              setup->datastart);
  modifyreg32(STM32L4_FIREWALL_VDSL, FIREWALL_VDSLENG_MASK,
              setup->datalen);

  /* Define access options */

  reg = getreg32(STM32L4_FIREWALL_CR);
  if (setup->datashared)
    {
      reg |= FIREWALL_CR_VDS;
    }

  if (setup->dataexec)
    {
      reg |= FIREWALL_CR_VDE;
    }

  putreg32(reg, STM32L4_FIREWALL_CR);

  /* Enable firewall */

  reg  = getreg32(STM32L4_SYSCFG_CFGR1);
  reg &= ~SYSCFG_CFGR1_FWDIS;
  putreg32(reg, STM32L4_SYSCFG_CFGR1);

  /* Now protected code can only be accessed by jumping to the FW gate */

  return 0;
}
