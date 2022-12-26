/****************************************************************************
 * arch/renesas/src/m16c/m16c_registerdump.c
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

#include "renesas_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

uintptr_t up_getusrsp(void)
{
  uint8_t *ptr = (uint8_t *) g_current_regs;
  return (uintptr_t)(ptr[REG_SP] << 8 | ptr[REG_SP + 1]);
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint8_t *ptr = dumpregs ? dumpregs : (uint8_t *)g_current_regs;

  /* Dump the interrupt registers */

  _alert("PC: %02x%02x%02x FLG: %02x00%02x FB: %02x%02x SB: %02x%02x "
         "SP: %02x%02x\n",
         ptr[REG_FLGPCHI] & 0xff, ptr[REG_PC], ptr[REG_PC + 1],
         ptr[REG_FLGPCHI] >> 8, ptr[REG_FLG],
         ptr[REG_FB], ptr[REG_FB + 1],
         ptr[REG_SB], ptr[REG_SB + 1],
         ptr[REG_SP], ptr[REG_SP + 1]);

  _alert("R0: %02x%02x R1: %02x%02x R2: %02x%02x A0: %02x%02x "
         "A1: %02x%02x\n",
         ptr[REG_R0], ptr[REG_R0 + 1], ptr[REG_R1], ptr[REG_R1 + 1],
         ptr[REG_R2], ptr[REG_R2 + 1], ptr[REG_R3], ptr[REG_R3 + 1],
         ptr[REG_A0], ptr[REG_A0 + 1], ptr[REG_A1], ptr[REG_A1 + 1]);
}
