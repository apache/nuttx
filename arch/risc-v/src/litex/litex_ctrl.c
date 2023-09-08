/****************************************************************************
 * arch/risc-v/src/litex/litex_ctrl.c
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

#include <stdint.h>

#include "hardware/litex_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LITEX_CTRL_RST_OFFSET             0x00
#define LITEX_CTRL_SCRATCH_OFFSET         0x04
#define LITEX_CTRL_BUS_ERROR_OFFSET       0x08

#define LITEX_CTRL_RST_RESET_BIT          0x00

#define CTRL_REG(X) (LITEX_CTRL_BASE + X)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: litex_ctrl_reset
 *
 * Description:
 *   Request a core reset.
 *
 * Returned Value:
 *  None: This function should not return.
 *
 ****************************************************************************/

void litex_ctrl_reset(void)
{
  putreg32(0x1 << LITEX_CTRL_RST_RESET_BIT, CTRL_REG(LITEX_CTRL_RST_OFFSET));
}

/****************************************************************************
 * Name: litex_ctrl_write_scratch
 *
 * Description:
 *   Write a value to the litex control block scratch register.
 *
 * Input Parameters:
 *   value  - The value to store in the register.
 *
 ****************************************************************************/

void litex_ctrl_write_scratch(uint32_t value)
{
  putreg32(value, CTRL_REG(LITEX_CTRL_SCRATCH_OFFSET));
}

/****************************************************************************
 * Name: litex_ctrl_read_scratch
 *
 * Description:
 *   Read a stored value from the litex control block scratch register.
 *
 * Returned Value:
 *  The value stored in the scratch register.
 *
 ****************************************************************************/

uint32_t litex_ctrl_read_scratch(void)
{
  return getreg32(CTRL_REG(LITEX_CTRL_SCRATCH_OFFSET));
}

/****************************************************************************
 * Name: litex_ctrl_read_bus_error
 *
 * Description:
 *   Read any bus error reported in the control block.
 *
 * Returned Value:
 *  The bus error number stored.
 *
 ****************************************************************************/

uint32_t litex_ctrl_read_bus_error(void)
{
  return getreg32(CTRL_REG(LITEX_CTRL_BUS_ERROR_OFFSET));
}
