/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_reset_ops.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/reset/reset-controller.h>

#include "riscv_internal.h"
#include "eic7700x_reset.h"
#include "hardware/eic7700x_reset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The manual gives no minimum pulse width for a software reset.  The
 * vendor Linux driver sleeps between ten and fifteen microseconds; a busy
 * wait of ten is used here instead so that the reset operation stays
 * callable from any context.
 */

#define EIC7700X_RESET_PULSE_US    10

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eic7700x_reset_decode
 *
 * Description:
 *   Split a reset line id into the register that holds the line and a
 *   single bit mask, and confirm that the bit names a line this SoC has.
 *
 *   Nothing in the reset framework bounds an id.  reset_control_get()
 *   rejects a negative index and hands every other value straight through,
 *   so this is the only place an out of range id is ever caught and every
 *   one of the operations below has to start here.
 *
 * Input Parameters:
 *   id   - The reset line id, register index times 32 plus bit
 *   reg  - Receives the address of the control register
 *   mask - Receives the bit mask within that register
 *
 * Returned Value:
 *   The register description, or NULL if the id names no reset line.
 *
 ****************************************************************************/

static FAR const struct eic7700x_reset_reg_s *
eic7700x_reset_decode(unsigned int id, FAR uintptr_t *reg,
                      FAR uint32_t *mask)
{
  FAR const struct eic7700x_reset_reg_s *desc;
  unsigned int index = EIC7700X_RESET_REGOF(id);
  uint32_t bit = 1ul << EIC7700X_RESET_BITOF(id);

  if (index >= EIC7700X_RESET_NREGS)
    {
      rsterr("id %u: register %u is past the end of the block\n",
             id, index);
      return NULL;
    }

  desc = &g_eic7700x_reset_regs[index];
  if ((desc->valid & bit) == 0)
    {
      rsterr("id %u: no reset line at register %u bit %u\n",
             id, index, EIC7700X_RESET_BITOF(id));
      return NULL;
    }

  *reg  = EIC7700X_RESET_CTRL(index);
  *mask = bit;
  return desc;
}

/****************************************************************************
 * Name: eic7700x_reset_status
 *
 * Description:
 *   Report whether a reset line is asserted.  The lines are active low, so
 *   a line is in reset while its bit reads zero.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *
 * Returned Value:
 *   One if the line is asserted, zero if it is released, or a negated
 *   errno if the id names no line.
 *
 ****************************************************************************/

static int eic7700x_reset_status(FAR struct reset_controller_dev *rcdev,
                                 unsigned int id)
{
  uintptr_t reg;
  uint32_t mask;

  if (eic7700x_reset_decode(id, &reg, &mask) == NULL)
    {
      return -EINVAL;
    }

  return (getreg32(reg) & mask) != 0 ? 0 : 1;
}

/****************************************************************************
 * Name: eic7700x_reset_deassert
 *
 * Description:
 *   Bring a line out of reset by setting its bit.  Deassert is allowed on
 *   every line the hardware lets software drive, including the lines that
 *   refuse to be asserted: releasing a reset can only ever start a block,
 *   never stop one.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *
 * Returned Value:
 *   OK, -EINVAL if the id names no line, or -EROFS if the hardware will
 *   not let software drive it.
 *
 ****************************************************************************/

static int eic7700x_reset_deassert(FAR struct reset_controller_dev *rcdev,
                                   unsigned int id)
{
  FAR const struct eic7700x_reset_reg_s *desc;
  uintptr_t reg;
  uint32_t mask;

  desc = eic7700x_reset_decode(id, &reg, &mask);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if ((desc->rdonly & mask) != 0)
    {
      rsterr("id %u: line reports state only, it cannot be driven\n", id);
      return -EROFS;
    }

  modifyreg32(reg, 0, mask);
  return OK;
}

/****************************************************************************
 * Name: eic7700x_reset_assert
 *
 * Description:
 *   Put a line into reset by clearing its bit, unless the line carries
 *   something the running system cannot lose.  See the policy note above
 *   the table in eic7700x_reset.c for what counts as that.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *
 * Returned Value:
 *   OK, -EINVAL if the id names no line, -EROFS if the hardware will not
 *   let software drive it, or -EPERM if the line is system critical.
 *
 ****************************************************************************/

static int eic7700x_reset_assert(FAR struct reset_controller_dev *rcdev,
                                 unsigned int id)
{
  FAR const struct eic7700x_reset_reg_s *desc;
  uintptr_t reg;
  uint32_t mask;

  desc = eic7700x_reset_decode(id, &reg, &mask);
  if (desc == NULL)
    {
      return -EINVAL;
    }

  if ((desc->rdonly & mask) != 0)
    {
      rsterr("id %u: line reports state only, it cannot be driven\n", id);
      return -EROFS;
    }

  if ((desc->critical & mask) != 0)
    {
      rsterr("id %u: refusing to assert a system critical line\n", id);
      return -EPERM;
    }

  modifyreg32(reg, mask, 0);
  return OK;
}

/****************************************************************************
 * Name: eic7700x_reset_reset
 *
 * Description:
 *   Pulse a line: assert, wait, deassert.
 *
 *   The assert goes through eic7700x_reset_assert() rather than being
 *   done inline so that the id checks and the refusal to touch a system
 *   critical line live in one place.  A line that must not be asserted
 *   must not be pulsed either, and this way that holds by construction.
 *
 *   The pair is not atomic against a concurrent assert on the same line.
 *   Each register update is, but the framework does not serialise
 *   operations and this provider does not add a lock the framework does
 *   not expect.
 *
 * Input Parameters:
 *   rcdev - The reset controller
 *   id    - The reset line id
 *
 * Returned Value:
 *   OK, or the error eic7700x_reset_assert() refused the line with; a
 *   line that may not be asserted may not be pulsed either.
 *
 ****************************************************************************/

static int eic7700x_reset_reset(FAR struct reset_controller_dev *rcdev,
                                unsigned int id)
{
  int ret;

  ret = eic7700x_reset_assert(rcdev, id);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(EIC7700X_RESET_PULSE_US);
  return eic7700x_reset_deassert(rcdev, id);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The acquire and release operations are deliberately absent.  The
 * framework only calls them when the rpmsg proxy is built and treats a
 * missing one as success, so there is nothing for them to do here.
 *
 * Naming a member assert in a designated initialiser is safe even though
 * assert() is a function like macro: such a macro only expands where the
 * identifier is followed by an opening parenthesis, and here it is
 * followed by an equals sign.  drivers/reset/core.c needs an #undef only
 * because it calls through this table.  Nothing in this file does, so
 * nothing here undefines assert(), which would disable it for the rest of
 * the translation unit.
 */

const struct reset_control_ops g_eic7700x_reset_ops =
{
  .reset    = eic7700x_reset_reset,
  .assert   = eic7700x_reset_assert,
  .deassert = eic7700x_reset_deassert,
  .status   = eic7700x_reset_status,
#ifdef CONFIG_RESET_PROCFS
  .get_line = eic7700x_reset_getline,
#endif
};
