/****************************************************************************
 * wireless/bluetooth/bt_atomic.c
 * Linux like atomic operations
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
#include <nuttx/spinlock.h>

#include "bt_atomic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bt_atomic_t bt_atomic_incr(FAR bt_atomic_t *ptr)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value + 1;
  spin_unlock_irqrestore(NULL, flags);

  return value;
}

bt_atomic_t bt_atomic_decr(FAR bt_atomic_t *ptr)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value - 1;
  spin_unlock_irqrestore(NULL, flags);

  return value;
}

bt_atomic_t bt_atomic_setbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value | (1 << bitno);
  spin_unlock_irqrestore(NULL, flags);

  return value;
}

bt_atomic_t bt_atomic_clrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value & ~(1 << bitno);
  spin_unlock_irqrestore(NULL, flags);

  return value;
}

bool bt_atomic_testsetbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value | (1 << bitno);
  spin_unlock_irqrestore(NULL, flags);

  return (value & (1 << bitno)) != 0;
}

bool bt_atomic_testclrbit(FAR bt_atomic_t *ptr, bt_atomic_t bitno)
{
  irqstate_t flags;
  bt_atomic_t value;

  flags = spin_lock_irqsave(NULL);
  value = *ptr;
  *ptr  = value & ~(1 << bitno);
  spin_unlock_irqrestore(NULL, flags);

  return (value & (1 << bitno)) != 0;
}
