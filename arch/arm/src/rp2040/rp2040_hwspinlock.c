/****************************************************************************
 * arch/arm/src/rp2040/rp2040_hwspinlock.c
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

#include <nuttx/hwspinlock/hwspinlock.h>

#include "arm_internal.h"
#include "hardware/rp2040_sio.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool rp2040_hwspinlock_trylock(struct hwspinlock_dev_s *dev);
static void rp2040_hwspinlock_unlock(struct hwspinlock_dev_s *dev);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct hwspinlock_ops_s g_rp2040_hwspinlock_ops =
{
  .trylock = rp2040_hwspinlock_trylock,
  .unlock  = rp2040_hwspinlock_unlock
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool rp2040_hwspinlock_trylock(struct hwspinlock_dev_s *dev)
{
  return getreg32(RP2040_SIO_SPINLOCK(dev->id));
}

static void rp2040_hwspinlock_unlock(struct hwspinlock_dev_s *dev)
{
  putreg32(0, RP2040_SIO_SPINLOCK(dev->id));
}
