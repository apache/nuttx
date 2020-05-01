/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_firewall.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>

#include <nuttx/arch.h>

#include "arm_arch.h"
#include "hardware/stm32l4_syscfg.h"

#include "stm32l4_firewall.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32l4_firewallsetup(FAR struct stm32l4_firewall_t *setup)
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
