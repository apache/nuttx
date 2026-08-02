/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_reset.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_EIC7700X_RESET_H
#define __ARCH_RISCV_SRC_EIC7700X_EIC7700X_RESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/reset/reset-controller.h>

#include <arch/chip/eic7700x_reset.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One reset control register, described by three masks over the same
 * thirty two bits.
 *
 * The sets nest: rdonly and critical are both subsets of valid, and they
 * do not overlap each other.  A line the hardware will not let software
 * drive is already unreachable, so marking it critical as well would say
 * nothing.  eic7700x_reset_initialize() checks all three invariants when
 * assertions are enabled.
 */

struct eic7700x_reset_reg_s
{
  uint32_t valid;        /* Bits that name a reset line                   */
  uint32_t rdonly;       /* Valid bits the hardware will not let us drive */
  uint32_t critical;     /* Valid bits that refuse assert and reset       */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const struct reset_control_ops g_eic7700x_reset_ops;
EXTERN const struct eic7700x_reset_reg_s
             g_eic7700x_reset_regs[EIC7700X_RESET_NREGS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_reset_initialize
 *
 * Description:
 *   Register the reset lines of the Clock and Reset Generator with the
 *   reset framework.  Registration writes nothing to the hardware: a line
 *   only moves when a driver asks it to.
 *
 * Returned Value:
 *   OK on success, or a negated errno on failure.
 *
 ****************************************************************************/

int eic7700x_reset_initialize(void);

/****************************************************************************
 * Name: eic7700x_reset_count
 *
 * Description:
 *   How many reset lines the controller has, and how many of them the boot
 *   loader left held.  Both zero before eic7700x_reset_initialize() has
 *   run.  The held figure is the state at that moment and does not track
 *   what drivers release afterwards.
 *
 * Input Parameters:
 *   held - Where to report the number held, or NULL
 *
 * Returned Value:
 *   The number of reset lines.
 *
 ****************************************************************************/

unsigned int eic7700x_reset_count(FAR unsigned int *held);

#ifdef CONFIG_RESET_PROCFS
/****************************************************************************
 * Name: eic7700x_reset_getline
 *
 * Description:
 *   Describe one reset line for /proc/reset.  Lives beside the name table
 *   rather than beside the operations, since that is what it reads.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *   info  - Receives the name, and the register and bit as extra text
 *
 * Returned Value:
 *   OK, or -ENODEV if the id names no line.
 *
 ****************************************************************************/

int eic7700x_reset_getline(FAR struct reset_controller_dev *rcdev,
                           unsigned int id,
                           FAR struct reset_lineinfo_s *info);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_EIC7700X_EIC7700X_RESET_H */
