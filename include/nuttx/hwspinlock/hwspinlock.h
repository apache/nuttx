/****************************************************************************
 * include/nuttx/hwspinlock/hwspinlock.h
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
#include <nuttx/spinlock.h>

#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hwspinlock_dev_s;

struct hwspinlock_ops_s
{
  CODE bool (*trylock)(FAR struct hwspinlock_dev_s *dev,
                       int id, int priority);
  CODE void (*relax)(FAR struct hwspinlock_dev_s *dev,
                     int id, int priority);
  CODE void (*unlock)(FAR struct hwspinlock_dev_s *dev, int id);
};

struct hwspinlock_dev_s
{
  spinlock_t lock;
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

static inline bool hwspin_trylock(FAR struct hwspinlock_dev_s *dev,
                                  int id, int priority)
{
  return dev->ops->trylock(dev, id, priority);
}

static inline bool hwspin_trylock_irqsave(FAR struct hwspinlock_dev_s *dev,
                                          int id, int priority,
                                          FAR irqstate_t *flags)
{
  *flags = spin_lock_irqsave(&dev->lock);
  if (hwspin_trylock(dev, id, priority))
    {
      return true;
    }

  spin_unlock_irqrestore(&dev->lock, *flags);
  return false;
}

static inline void hwspin_lock(FAR struct hwspinlock_dev_s *dev,
                               int id, int priority)
{
  while (!dev->ops->trylock(dev, id, priority))
    {
      if (dev->ops->relax)
        {
          dev->ops->relax(dev, id, priority);
        }
    }
}

static inline irqstate_t
hwspin_lock_irqsave(FAR struct hwspinlock_dev_s *dev,
                    int id, int priority)
{
  irqstate_t flags = spin_lock_irqsave(&dev->lock);
  hwspin_lock(dev, id, priority);
  return flags;
}

static inline void hwspin_unlock(FAR struct hwspinlock_dev_s *dev,
                                 int id, int priority)
{
  dev->ops->unlock(dev, id);
}

static inline void hwspin_unlock_restore(FAR struct hwspinlock_dev_s *dev,
                                         int id, int priority,
                                         irqstate_t flags)
{
  hwspin_unlock(dev, id);
  spin_lock_restore(&dev->lock, flags);
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HWSPINLOCK_HWSPINLOCK_H */
