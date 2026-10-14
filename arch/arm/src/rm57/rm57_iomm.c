/****************************************************************************
 * arch/arm/src/rm57/rm57_iomm.c
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

/* Pin multiplexing for the I/O Multiplexing and Control Module (IOMM).
 *
 * This replaces HALCoGen's muxInit(), which writes every
 * PINMMR register from a fixed, generated table.  Here a board lists only
 * the pins it re-purposes, in BOARD_PINMUX_INITIALIZER, and every other
 * ball keeps its reset function.
 *
 * The PINMMR registers are write protected: they accept writes only while
 * both kicker registers hold their unlock values, so each update sequence
 * below unlocks them, writes, and locks them again.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>
#include <stdint.h>

#include <nuttx/spinlock.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rm57_iomm.h"
#include "hardware/rm57_pinmux.h"
#include "rm57_iomm.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serializes the unlock/modify/lock sequences, which are not otherwise
 * safe against concurrent updates of two fields in one PINMMR register.
 */

static spinlock_t g_rm57_iomm_lock = SP_UNLOCKED;

#ifdef BOARD_PINMUX_INITIALIZER
static const struct rm57_pinmux_s g_pinmux_table[] =
{
  BOARD_PINMUX_INITIALIZER
};

#  define NPINMUX (sizeof(g_pinmux_table) / sizeof(struct rm57_pinmux_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_iomm_unlock
 *
 * Description:
 *   Enable writes to the PINMMR registers.
 *
 ****************************************************************************/

static void rm57_iomm_unlock(void)
{
  putreg32(IOMM_KICK0_UNLOCK, RM57_IOMM_KICK0);
  putreg32(IOMM_KICK1_UNLOCK, RM57_IOMM_KICK1);
}

/****************************************************************************
 * Name: rm57_iomm_lock
 *
 * Description:
 *   Restore the write protection of the PINMMR registers.
 *
 ****************************************************************************/

static void rm57_iomm_lock(void)
{
  putreg32(IOMM_KICK0_LOCK, RM57_IOMM_KICK0);
  putreg32(IOMM_KICK1_LOCK, RM57_IOMM_KICK1);
}

/****************************************************************************
 * Name: rm57_iomm_setfield
 *
 * Description:
 *   Write one 8-bit field of one PINMMR register.  The kicker registers
 *   must already be unlocked.
 *
 ****************************************************************************/

static void rm57_iomm_setfield(const struct rm57_pinmux_s *pin)
{
  uintptr_t regaddr;
  uint32_t  regval;

  regaddr = RM57_IOMM_PINMMR(pin->mmrndx);
  regval  = getreg32(regaddr);
  regval &= ~((uint32_t)0xff << pin->shift);
  regval |= ((uint32_t)(pin->value) << pin->shift);
  putreg32(regval, regaddr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_iomm_config
 *
 * Description:
 *   Select the function of a single pin, input signal or special-function
 *   field.
 *
 ****************************************************************************/

void rm57_iomm_config(const struct rm57_pinmux_s *pin)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_rm57_iomm_lock);

  rm57_iomm_unlock();
  rm57_iomm_setfield(pin);
  rm57_iomm_lock();

  spin_unlock_irqrestore(&g_rm57_iomm_lock, flags);
}

/****************************************************************************
 * Name: rm57_iomm_configmultiple
 *
 * Description:
 *   Apply a table of pin function selections, unlocking the kicker
 *   registers once for the whole table.
 *
 ****************************************************************************/

void rm57_iomm_configmultiple(const struct rm57_pinmux_s *table,
                              size_t count)
{
  irqstate_t flags;
  size_t i;

  if (count == 0)
    {
      return;
    }

  flags = spin_lock_irqsave(&g_rm57_iomm_lock);

  rm57_iomm_unlock();

  for (i = 0; i < count; i++)
    {
      rm57_iomm_setfield(&table[i]);
    }

  rm57_iomm_lock();

  spin_unlock_irqrestore(&g_rm57_iomm_lock, flags);
}

/****************************************************************************
 * Name: rm57_iomm_initialize
 *
 * Description:
 *   Apply the board pin multiplexing described by BOARD_PINMUX_INITIALIZER.
 *
 ****************************************************************************/

void rm57_iomm_initialize(void)
{
#ifdef BOARD_PINMUX_INITIALIZER
  rm57_iomm_configmultiple(g_pinmux_table, NPINMUX);
#endif
}
