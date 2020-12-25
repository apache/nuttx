/****************************************************************************
 * wireless/bluetooth/bt_atomic.c
 * Linux like atomic operations
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/irq.h>

#include "bt_atomic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bt_atomic_t bt_atomic_incr(FAR bt_atomic_t *ptr)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value + 1;
  spin_unlock_irqrestore(flags);

  return value;
}

bt_atomic_t bt_atomic_decr(FAR bt_atomic_t *ptr)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value - 1;
  spin_unlock_irqrestore(flags);

  return value;
}

bt_atomic_t bt_atomic_setbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value | (1 << bitno);
  spin_unlock_irqrestore(flags);

  return value;
}

bt_atomic_t bt_atomic_clrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value & ~(1 << bitno);
  spin_unlock_irqrestore(flags);

  return value;
}

bool bt_atomic_testsetbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value | (1 << bitno);
  spin_unlock_irqrestore(flags);

  return (value & (1 << bitno)) != 0;
}

bool bt_atomic_testclrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave();
  value = *ptr;
  *ptr  = value & ~(1 << bitno);
  spin_unlock_irqrestore(flags);

  return (value & (1 << bitno)) != 0;
}
