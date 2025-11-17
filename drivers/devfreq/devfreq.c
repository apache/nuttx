/****************************************************************************
 * drivers/devfreq/devfreq.c
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

#include <debug.h>

#include <nuttx/devfreq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node g_devfreq_list = LIST_INITIAL_VALUE(g_devfreq_list);
static spinlock_t g_devfreq_list_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int devfreq_init_governor(FAR struct devfreq_s *devfreq);
static void devfreq_exit_governor(FAR struct devfreq_s *devfreq);
static int devfreq_start_governor(FAR struct devfreq_s *devfreq);
static void devfreq_stop_governor(FAR struct devfreq_s *devfreq);
static void devfreq_limit_governor(FAR struct devfreq_s *devfreq);
static ssize_t devfreq_table_find_freq(FAR struct devfreq_s *devfreq,
                                       uint32_t target_freq,
                                       int relation);
static int devfreq_table_validate(FAR struct devfreq_s *devfreq);
static void devfreq_refresh_limit(FAR struct devfreq_s *devfreq);
static int devfreq_driver_target(FAR struct devfreq_s *devfreq,
                                 uint32_t target_freq,
                                 int relation);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devfreq_init_governor
 *
 * Description:
 *   Initialize governor
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int devfreq_init_governor(FAR struct devfreq_s *devfreq)
{
  if (!devfreq->governor)
    {
      return -EINVAL;
    }

  if (devfreq->governor->init)
    {
      return devfreq->governor->init(devfreq);
    }

  return 0;
}

/****************************************************************************
 * Name: devfreq_exit_governor
 *
 * Description:
 *   Exit governor
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void devfreq_exit_governor(FAR struct devfreq_s *devfreq)
{
  if (!devfreq->governor)
    {
      return;
    }

  if (devfreq->governor->exit)
    {
      devfreq->governor->exit(devfreq);
    }
}

/****************************************************************************
 * Name: devfreq_start_governor
 *
 * Description:
 *   Start governor
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int devfreq_start_governor(FAR struct devfreq_s *devfreq)
{
  if (devfreq->suspended)
    {
      return 0;
    }

  if (!devfreq->governor)
    {
      return -EINVAL;
    }

  if (devfreq->governor->start)
    {
      int ret = devfreq->governor->start(devfreq);
      if (ret < 0)
        {
          return ret;
        }
    }

  devfreq_limit_governor(devfreq);
  return 0;
}

/****************************************************************************
 * Name: devfreq_stop_governor
 *
 * Description:
 *   Stop governor
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static void devfreq_stop_governor(FAR struct devfreq_s *devfreq)
{
  if (devfreq->suspended || !devfreq->governor)
    {
      return;
    }

  if (devfreq->governor->stop)
    {
      devfreq->governor->stop(devfreq);
    }
}

/****************************************************************************
 * Name: devfreq_limit_governor
 *
 * Description:
 *   Limit governor
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static void devfreq_limit_governor(FAR struct devfreq_s *devfreq)
{
  if (devfreq->suspended || !devfreq->governor)
    {
      return;
    }

  if (devfreq->governor->limit)
    {
      uint32_t freq = devfreq->governor->limit(devfreq);
      devfreq_driver_target(devfreq, freq, DEVFREQ_RELATION_L);
    }
}

/****************************************************************************
 * Name: devfreq_table_find_freq
 *
 * Description:
 *   Find frequency in table
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *   target_freq - target frequency
 *   relation - relation
 *
 * Returned Value:
 *   target index on success; negated errno on failure
 *
 ****************************************************************************/

static ssize_t devfreq_table_find_freq(FAR struct devfreq_s *devfreq,
                                       uint32_t target_freq,
                                       int relation)
{
  ssize_t best = -ENOENT;
  size_t i;

  if (relation == DEVFREQ_RELATION_L)
    {
      for (i = 0; devfreq->freq_table[i] != DEVFREQ_ENTRY_END; i++)
        {
          if (devfreq->freq_table[i] == DEVFREQ_ENTRY_INVALID)
            {
              continue;
            }

          if (devfreq->freq_table[i] >= target_freq)
            {
              best = i;
              break;
            }
        }
    }
  else if (relation == DEVFREQ_RELATION_H)
    {
      for (i = 0; devfreq->freq_table[i] != DEVFREQ_ENTRY_END; i++)
        {
          if (devfreq->freq_table[i] == DEVFREQ_ENTRY_INVALID)
            {
              continue;
            }

          if (devfreq->freq_table[i] > target_freq)
            {
              break;
            }

          best = i;
        }
    }

  return best;
}

/****************************************************************************
 * Name: devfreq_table_validate
 *
 * Description:
 *   Validate table
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int devfreq_table_validate(FAR struct devfreq_s *devfreq)
{
  FAR const uint32_t *table = devfreq->freq_table;
  uint32_t prv_freq         = 0;
  uint32_t min_freq         = UINT32_MAX;
  uint32_t max_freq         = 0;
  size_t i;

  if (!table)
    {
      return -EINVAL;
    }

  for (i = 0; table[i] != DEVFREQ_ENTRY_END; i++)
    {
      if (table[i] == DEVFREQ_ENTRY_INVALID)
        {
          continue;
        }

      if (i && table[i] <= prv_freq)
        {
          return -EINVAL;
        }

      if (table[i] < min_freq)
        {
          min_freq = table[i];
        }

      if (table[i] > max_freq)
        {
          max_freq = table[i];
        }

      prv_freq = table[i];
    }

  devfreq->min = min_freq;
  devfreq->max = max_freq;

  return 0;
}

/****************************************************************************
 * Name: devfreq_refresh_limit
 *
 * Description:
 *   Refresh limit
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void devfreq_refresh_limit(FAR struct devfreq_s *devfreq)
{
  uint32_t min;
  uint32_t max;
  ssize_t idx;

  min = qos_get_value(&devfreq->constraints, QOS_REQ_MIN);
  max = qos_get_value(&devfreq->constraints, QOS_REQ_MAX);

  if (min > max)
    {
      min = max;
    }

  idx = devfreq_table_find_freq(devfreq, min, DEVFREQ_RELATION_L);
  if (idx >= 0)
    {
      devfreq->min = devfreq->freq_table[idx];
    }

  idx = devfreq_table_find_freq(devfreq, max, DEVFREQ_RELATION_H);
  if (idx >= 0)
    {
      devfreq->max = devfreq->freq_table[idx];
    }

  devfreq_limit_governor(devfreq);
}

/****************************************************************************
 * Name: devfreq_driver_target
 *
 * Description:
 *   Set target frequency
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *   target_freq - target frequency
 *   relation - relation to target frequency
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int devfreq_driver_target(FAR struct devfreq_s *devfreq,
                                 uint32_t target_freq,
                                 int relation)
{
  struct devfreq_notifier_s freq;
  ssize_t idx;
  int ret;

  if (!devfreq)
    {
      return -EINVAL;
    }

  idx = devfreq_table_find_freq(devfreq, target_freq, relation);
  if (idx < 0)
    {
      return -ENOENT;
    }

  target_freq = devfreq->freq_table[idx];
  if (target_freq == devfreq->cur)
    {
      return 0;
    }

  freq.old = devfreq->cur;
  freq.new = target_freq;

  blocking_notifier_call_chain(&devfreq->notifier_list,
                               DEVFREQ_PRECHANGE, &freq);
  ret = devfreq->driver->target_index(devfreq, idx);
  blocking_notifier_call_chain(&devfreq->notifier_list,
                               DEVFREQ_POSTCHANGE, &freq);
  if (ret < 0)
    {
      freq.old = target_freq;
      freq.new = devfreq->cur;
      blocking_notifier_call_chain(&devfreq->notifier_list,
                                   DEVFREQ_PRECHANGE, &freq);
      blocking_notifier_call_chain(&devfreq->notifier_list,
                                   DEVFREQ_POSTCHANGE, &freq);
      return ret;
    }

  devfreq->cur = target_freq;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devfreq_register
 *
 * Description:
 *   Register devfreq device
 *
 * Input Parameters:
 *   name - device name
 *   governor - governor
 *   driver - driver
 *   priv - private data
 *
 * Returned Value:
 *   devfreq device on success; NULL on failure
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_register(
                          FAR const char *name,
                          FAR const struct devfreq_governor_s *governor,
                          FAR const struct devfreq_driver_s *driver,
                          FAR void *priv)
{
  FAR struct devfreq_s *devfreq = devfreq_find_by_name(name);
  irqstate_t flags;

  if (devfreq || !driver)
    {
      return NULL;
    }

  devfreq = (FAR struct devfreq_s *)kmm_zalloc(sizeof(struct devfreq_s));
  if (!devfreq)
    {
      return NULL;
    }

  qos_constraints_init(&devfreq->constraints);
  BLOCKING_INIT_NOTIFIER_HEAD(&devfreq->notifier_list);
  nxmutex_init(&devfreq->lock);

  strlcpy(devfreq->name, name, NAME_MAX);
  devfreq->driver     = driver;
  devfreq->priv       = priv;
  devfreq->suspended  = false;
  devfreq->freq_table = driver->get_table(devfreq);
  devfreq->min        = 0;
  devfreq->max        = UINT32_MAX;
  devfreq->cur        = driver->get_frequency(devfreq);
  if (!devfreq->freq_table)
    {
      goto out;
    }

  if (devfreq_table_validate(devfreq) < 0)
    {
      goto out;
    }

  if (!governor)
    {
      goto out;
    }

  devfreq->governor = governor;

  if (devfreq_init_governor(devfreq) < 0)
    {
      goto out;
    }

  devfreq_start_governor(devfreq);

  flags = spin_lock_irqsave(&g_devfreq_list_lock);
  list_add_tail(&g_devfreq_list, &devfreq->node);
  spin_unlock_irqrestore(&g_devfreq_list_lock, flags);

  return devfreq;

out:
  nxmutex_destroy(&devfreq->lock);
  nxmutex_destroy(&devfreq->notifier_list.mutex);
  kmm_free(devfreq);
  return NULL;
}

/****************************************************************************
 * Name: devfreq_unregister
 *
 * Description:
 *   Unregister devfreq device
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_unregister(FAR struct devfreq_s *devfreq)
{
  irqstate_t flags;

  if (!devfreq)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_devfreq_list_lock);
  list_delete(&devfreq->node);
  spin_unlock_irqrestore(&g_devfreq_list_lock, flags);

  devfreq_stop_governor(devfreq);
  devfreq_exit_governor(devfreq);

  nxmutex_destroy(&devfreq->lock);
  nxmutex_destroy(&devfreq->notifier_list.mutex);
  kmm_free(devfreq);
  return 0;
}

/****************************************************************************
 * Name: devfreq_suspend
 *
 * Description:
 *   Suspend devfreq device
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_suspend(FAR struct devfreq_s *devfreq)
{
  nxmutex_lock(&devfreq->lock);

  devfreq_stop_governor(devfreq);

  if (devfreq->driver->suspend)
    {
      int ret = devfreq->driver->suspend(devfreq);
      if (ret < 0)
        {
          nxmutex_unlock(&devfreq->lock);
          return ret;
        }
    }

  devfreq->suspended = true;
  nxmutex_unlock(&devfreq->lock);
  return 0;
}

/****************************************************************************
 * Name: devfreq_resume
 *
 * Description:
 *   Resume devfreq device
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_resume(struct devfreq_s *devfreq)
{
  nxmutex_lock(&devfreq->lock);

  if (devfreq->driver->resume)
    {
      int ret = devfreq->driver->resume(devfreq);
      if (ret < 0)
        {
          nxmutex_unlock(&devfreq->lock);
          return ret;
        }
    }

  devfreq->suspended = false;
  devfreq_start_governor(devfreq);

  nxmutex_unlock(&devfreq->lock);
  return 0;
}

/****************************************************************************
 * Name: devfreq_register_notifier
 *
 * Description:
 *   Register notifier
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *   nb - notifier block
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_register_notifier(FAR struct devfreq_s *devfreq,
                              FAR struct notifier_block *nb)
{
  if (!devfreq || !nb)
    {
      return -EINVAL;
    }

  blocking_notifier_chain_register(&devfreq->notifier_list, nb);
  return 0;
}

/****************************************************************************
 * Name: devfreq_unregister_notifier
 *
 * Description:
 *   Unregister notifier
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *   nb - notifier block
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_unregister_notifier(FAR struct devfreq_s *devfreq,
                                FAR struct notifier_block *nb)
{
  if (!devfreq || !nb)
    {
      return -EINVAL;
    }

  blocking_notifier_chain_unregister(&devfreq->notifier_list, nb);
  return 0;
}

/****************************************************************************
 * Name: devfreq_get_frequency
 *
 * Description:
 *   Get current frequency
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *
 * Returned Value:
 *   Current frequency
 *
 ****************************************************************************/

FAR uint32_t devfreq_get_frequency(FAR struct devfreq_s *devfreq)
{
  return devfreq->driver->get_frequency(devfreq);
}

/****************************************************************************
 * Name: devfreq_qos_add_request
 *
 * Description:
 *   Add a new request
 *
 * Input Parameters:
 *   devfreq - devfreq device
 *   min - minimum frequency
 *   max - maximum frequency
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

FAR struct qos_request_s *devfreq_qos_add_request(
                            FAR struct devfreq_s *devfreq,
                            uint32_t min, uint32_t max)
{
  FAR struct qos_request_s *req;

  if (!devfreq)
    {
      return NULL;
    }

  nxmutex_lock(&devfreq->lock);

  req = qos_add_request(&devfreq->constraints, min, max);
  devfreq_refresh_limit(devfreq);

  nxmutex_unlock(&devfreq->lock);
  return req;
}

/****************************************************************************
 * Name: devfreq_qos_update_request
 *
 * Description:
 *   Update a request
 *
 * Input Parameters:
 *   qos - devfreq qos
 *   min - minimum frequency
 *   max - maximum frequency
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_qos_update_request(FAR struct devfreq_s *devfreq,
                               FAR struct qos_request_s *req,
                               uint32_t min, uint32_t max)
{
  int ret;

  if (!devfreq || !req)
    {
      return -EINVAL;
    }

  nxmutex_lock(&devfreq->lock);

  ret = qos_update_request(&devfreq->constraints, req, min, max);
  if (ret < 0)
    {
      nxmutex_unlock(&devfreq->lock);
      return ret;
    }

  devfreq_refresh_limit(devfreq);

  nxmutex_unlock(&devfreq->lock);
  return ret;
}

/****************************************************************************
 * Name: devfreq_qos_remove_request
 *
 * Description:
 *   Remove a request
 *
 * Input Parameters:
 *   qos - devfreq qos
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int devfreq_qos_remove_request(FAR struct devfreq_s *devfreq,
                               FAR struct qos_request_s *req)
{
  int ret;

  if (!devfreq || !req)
    {
      return -EINVAL;
    }

  nxmutex_lock(&devfreq->lock);

  ret = qos_remove_request(&devfreq->constraints, req);
  if (ret < 0)
    {
      nxmutex_unlock(&devfreq->lock);
      return ret;
    }

  devfreq_refresh_limit(devfreq);

  nxmutex_unlock(&devfreq->lock);
  return ret;
}

/****************************************************************************
 * Name: devfreq_find_by_name
 *
 * Description:
 *   find a devfreq entry from global list by name
 *
 * Input Parameters:
 *   name - devfreq name
 *
 * Returned Value:
 *   devfreq handle
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_find_by_name(FAR const char *name)
{
  FAR struct devfreq_s *devfreq;
  irqstate_t flags;

  if (!name)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&g_devfreq_list_lock);

  list_for_every_entry(&g_devfreq_list, devfreq, struct devfreq_s, node)
    {
      if (!strcmp(devfreq->name, name))
        {
          spin_unlock_irqrestore(&g_devfreq_list_lock, flags);
          return devfreq;
        }
    }

  spin_unlock_irqrestore(&g_devfreq_list_lock, flags);
  return NULL;
}

/****************************************************************************
 * Name: devfreq_find_by_index
 *
 * Description:
 *   find a devfreq entry from global list by index
 *
 * Input Parameters:
 *   index - devfreq index
 *
 * Returned Value:
 *   devfreq handle
 *
 ****************************************************************************/

FAR struct devfreq_s *devfreq_find_by_index(size_t index)
{
  FAR struct devfreq_s *devfreq;
  irqstate_t flags;
  size_t i = 0;

  flags = spin_lock_irqsave(&g_devfreq_list_lock);

  list_for_every_entry(&g_devfreq_list, devfreq, struct devfreq_s, node)
    {
      if (index == i++)
        {
          spin_unlock_irqrestore(&g_devfreq_list_lock, flags);
          return devfreq;
        }
    }

  spin_unlock_irqrestore(&g_devfreq_list_lock, flags);
  return NULL;
}
