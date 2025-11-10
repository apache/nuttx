/****************************************************************************
 * include/nuttx/hwspinlock/hwspinlock.h
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

#ifndef __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H
#define __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hwspinlock_dev_s;

struct hwspinlock_ops_s
{
  CODE bool (*trylock)(FAR struct hwspinlock_dev_s *dev);
  CODE void (*relax)(FAR struct hwspinlock_dev_s *dev);
  CODE void (*unlock)(FAR struct hwspinlock_dev_s *dev);
};

struct hwspinlock_dev_s
{
  int id;
  int priority;
  FAR const struct hwspinlock_ops_s *ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

static inline bool hwspin_trylock(FAR struct hwspinlock_dev_s *dev)
{
  return dev->ops->trylock(dev);
}

static inline bool hwspin_trylock_irqsave(FAR struct hwspinlock_dev_s *dev,
                                          FAR irqstate_t *flags)
{
  *flags = up_irq_save();
  if (hwspin_trylock(dev))
    {
      return true;
    }

  up_irq_restore(*flags);
  return false;
}

static inline void hwspin_lock(FAR struct hwspinlock_dev_s *dev)
{
  while (!dev->ops->trylock(dev))
    {
      if (dev->ops->relax)
        {
          dev->ops->relax(dev);
        }
    }
}

static inline irqstate_t
hwspin_lock_irqsave(FAR struct hwspinlock_dev_s *dev)
{
  irqstate_t flags = up_irq_save();
  hwspin_lock(dev);
  return flags;
}

static inline void hwspin_unlock(FAR struct hwspinlock_dev_s *dev)
{
  dev->ops->unlock(dev);
}

static inline void hwspin_unlock_restore(FAR struct hwspinlock_dev_s *dev,
                                         irqstate_t flags)
{
  hwspin_unlock(dev);
  up_irq_restore(flags);
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H */
