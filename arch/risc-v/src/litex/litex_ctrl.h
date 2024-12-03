/****************************************************************************
 * arch/risc-v/src/litex/litex_ctrl.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_LITEX_CTRL_H
#define __ARCH_RISCV_SRC_LITEX_LITEX_CTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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

void litex_ctrl_reset(void) noreturn_function;

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

void litex_ctrl_write_scratch(uint32_t value);

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

uint32_t litex_ctrl_read_scratch(void);

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

uint32_t litex_ctrl_read_bus_error(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_LITEX_LITEX_CTRL_H */
