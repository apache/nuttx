/****************************************************************************
 * arch/hc/src/m9s12/m9s12_registerdump.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "hc_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void *regs)
{
  uint8_t *ptr = regs;
  return (uintptr_t)(ptr[REG_SPH] << 8 | ptr[REG_SPL]);
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint8_t *regs = dumpregs ? dumpregs : (uint8_t *)g_current_regs;

  _alert("A:%02x B:%02x X:%02x%02x Y:%02x%02x PC:%02x%02x CCR:%02x\n",
         regs[REG_A],  regs[REG_B],  regs[REG_XH],  regs[REG_XL],
         regs[REG_YH], regs[REG_YL], regs[REG_PCH], regs[REG_PCL],
         regs[REG_CCR]);
  _alert("SP:%02x%02x FRAME:%02x%02x TMP:%02x%02x Z:%02x%02x XY:%02x\n",
         regs[REG_SPH],  regs[REG_SPL],  regs[REG_FRAMEH], regs[REG_FRAMEL],
         regs[REG_TMPL], regs[REG_TMPH], regs[REG_ZL],     regs[REG_ZH],
         regs[REG_XY],   regs[REG_XY + 1]);

#if CONFIG_HCS12_MSOFTREGS > 2
#  error "Need to save more registers"
#elif CONFIG_HCS12_MSOFTREGS == 2
  _alert("SOFTREGS: %02x%02x :%02x%02x\n",
         regs[REG_SOFTREG1], regs[REG_SOFTREG1 + 1],
         regs[REG_SOFTREG2], regs[REG_SOFTREG2 + 1]);
#elif CONFIG_HCS12_MSOFTREGS == 1
  _alert("SOFTREGS: %02x%02x\n",
         regs[REG_SOFTREG1], regs[REG_SOFTREG1 + 1]);
#endif

#ifndef CONFIG_HCS12_NONBANKED
  _alert("PPAGE: %02x\n", regs[REG_PPAGE]);
#endif
}
