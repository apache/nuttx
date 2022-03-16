/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_pcc.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "rv32m1_pcc.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_pcc_clock_enable
 ****************************************************************************/

void rv32m1_pcc_clock_enable(uint32_t regaddr)
{
  uint32_t regval = getreg32(regaddr);
  regval |= PCC_CLKCFG_CGC;
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Name: rv32m1_pcc_clock_disable
 ****************************************************************************/

void rv32m1_pcc_clock_disable(uint32_t regaddr)
{
  uint32_t regval = getreg32(regaddr);
  regval &= ~PCC_CLKCFG_CGC;
  putreg32(regval, regaddr);
}
