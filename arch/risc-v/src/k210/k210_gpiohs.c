/****************************************************************************
 * arch/risc-v/src/k210/k210_gpiohs.h
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
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

#include "riscv_arch.h"

#include "k210_memorymap.h"
#include "k210_gpiohs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIOHS_INPUT_VAL_OFFSET    0x00
#define GPIOHS_INPUT_EN_OFFSET     0x04
#define GPIOHS_OUTPUT_EN_OFFSET    0x08
#define GPIOHS_OUTPUT_VAL_OFFSET   0x0c
#define GPIOHS_PULLUP_EN_OFFSET    0x10
#define GPIOHS_DRIVE_OFFSET        0x14

#define GPIOHS_INPUT      (K210_GPIOHS_BASE + GPIOHS_INPUT_VAL_OFFSET)
#define GPIOHS_INPUT_EN   (K210_GPIOHS_BASE + GPIOHS_INPUT_EN_OFFSET)
#define GPIOHS_OUTPUT     (K210_GPIOHS_BASE + GPIOHS_OUTPUT_VAL_OFFSET)
#define GPIOHS_OUTPUT_EN  (K210_GPIOHS_BASE + GPIOHS_OUTPUT_EN_OFFSET)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void k210_gpiohs_set_direction(uint32_t io, bool dir)
{
  uint32_t outbit = dir << io;
  uint32_t inbit  = (!dir) << io;
  modifyreg32(GPIOHS_OUTPUT_EN, inbit, outbit);
  modifyreg32(GPIOHS_INPUT_EN, outbit, inbit);
}

void k210_gpiohs_set_value(uint32_t io, bool val)
{
  uint32_t setbit = val << io;
  uint32_t clrbit  = (!val) << io;
  modifyreg32(GPIOHS_OUTPUT, clrbit, setbit);
}

bool k210_gpiohs_get_value(uint32_t io)
{
  uint32_t reg = getreg32(GPIOHS_INPUT);

  if (reg & (1 << io))
    {
      return true;
    }
  else
    {
      return false;
    }
}
