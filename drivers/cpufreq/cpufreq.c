/****************************************************************************
 * drivers/cpufreq/cpufreq.c
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

/* CPU frequency scaling: one policy, arbitrated by windows.
 *
 * Each requester (a thermal cooling device, a holder of /dev/cpufreq, a
 * power manager) installs a [min, max] window.  The resolved frequency is
 * the highest table entry in the intersection of every window, so speed is
 * the default and any one requester can cap it.  When the windows do not
 * intersect the lowest maximum wins.
 *
 * The lower half only ever hears "go to table entry N".
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>

#include <syslog.h>

#include <nuttx/cpufreq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CPUFREQ_CHARDEV
static int     cpufreq_open(FAR struct file *filep);
static int     cpufreq_close(FAR struct file *filep);
static int     cpufreq_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct cpufreq_policy *g_cpufreq_policy;

#ifdef CONFIG_CPUFREQ_CHARDEV
static const struct file_operations g_cpufreq_fops =
{
  cpufreq_open,    /* open */
  cpufreq_close,   /* close */
  NULL,            /* read */
  NULL,            /* write */
  NULL,            /* seek */
  cpufreq_ioctl,   /* ioctl */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cpufreq_resolve
 *
 * Description:
 *   Recompute the target from every installed request and apply it if it
 *   moved.  Called with the policy lock held.
 *
 ****************************************************************************/

static int cpufreq_resolve(FAR struct cpufreq_policy *policy)
{
  FAR struct cpufreq_qos *qos;
  FAR dq_entry_t *entry;
  unsigned int hi = ~0u;
  unsigned int best;
  unsigned int i;
  int ret = OK;

  /* Only the ceilings matter to the pick: the table ascends, so the
   * highest entry under the lowest ceiling honours any satisfiable floor
   * automatically, and a floor that collides with a ceiling loses.
   */

  for (entry = dq_peek(&policy->requests); entry != NULL;
       entry = dq_next(entry))
    {
      qos = container_of(entry, struct cpufreq_qos, node);

      if (qos->max != CPUFREQ_NO_LIMIT && qos->max < hi)
        {
          hi = qos->max;
        }
    }

  best = 0;
  for (i = 0; i < policy->nentries; i++)
    {
      if (policy->table[i].frequency <= hi)
        {
          best = i;
        }
    }

  if (best != policy->current && !policy->suspended)
    {
      ret = policy->driver->target_index(policy, best);
      if (ret >= 0)
        {
          policy->current = best;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: File operations for /dev/cpufreq
 *
 * Description:
 *   Each open descriptor owns at most one request, installed by ioctl
 *   and withdrawn on clear or close, so a request is released when its
 *   descriptor closes, including on task exit.
 *
 ****************************************************************************/

#ifdef CONFIG_CPUFREQ_CHARDEV

static int cpufreq_open(FAR struct file *filep)
{
  filep->f_priv = NULL;
  return OK;
}

static int cpufreq_close(FAR struct file *filep)
{
  FAR struct cpufreq_qos *qos = filep->f_priv;

  if (qos != NULL)
    {
      cpufreq_qos_remove_request(qos);
      filep->f_priv = NULL;
    }

  return OK;
}

static int cpufreq_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct cpufreq_policy *policy = g_cpufreq_policy;
  FAR struct cpufreq_qos *qos = filep->f_priv;
  int ret = OK;

  if (policy == NULL)
    {
      return -ENODEV;
    }

  switch (cmd)
    {
      case CPUFREQIOC_GET_FREQUENCY:
        {
          FAR unsigned int *freq = (FAR unsigned int *)(uintptr_t)arg;

          if (freq == NULL)
            {
              return -EINVAL;
            }

          if (policy->driver->get_frequency != NULL)
            {
              ret = policy->driver->get_frequency(policy);
              if (ret < 0)
                {
                  return ret;
                }

              *freq = (unsigned int)ret;
              ret = OK;
            }
          else
            {
              *freq = policy->table[policy->current].frequency;
            }
        }
        break;

      case CPUFREQIOC_SET_REQUEST:
        {
          FAR const struct cpufreq_request_s *req =
            (FAR const struct cpufreq_request_s *)(uintptr_t)arg;

          if (req == NULL)
            {
              return -EINVAL;
            }

          if (qos != NULL)
            {
              ret = cpufreq_qos_update_request(qos, req->min, req->max);
            }
          else
            {
              qos = cpufreq_qos_add_request(policy, req->min, req->max);
              if (qos == NULL)
                {
                  return -ENOMEM;
                }

              filep->f_priv = qos;
            }
        }
        break;

      case CPUFREQIOC_CLEAR_REQUEST:
        {
          if (qos != NULL)
            {
              ret = cpufreq_qos_remove_request(qos);
              filep->f_priv = NULL;
            }
        }
        break;

      case CPUFREQIOC_GET_TABLE:
        {
          FAR struct cpufreq_table_query_s *query =
            (FAR struct cpufreq_table_query_s *)(uintptr_t)arg;
          unsigned int i;

          if (query == NULL)
            {
              return -EINVAL;
            }

          query->nentries = policy->nentries;
          if (query->frequencies != NULL)
            {
              for (i = 0; i < policy->nentries && i < query->maxlen; i++)
                {
                  query->frequencies[i] = policy->table[i].frequency;
                }
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#endif /* CONFIG_CPUFREQ_CHARDEV */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cpufreq_init
 ****************************************************************************/

int cpufreq_init(FAR struct cpufreq_driver *driver)
{
  FAR struct cpufreq_policy *policy;
  FAR const struct cpufreq_frequency_table *table;
  unsigned int count;
  int ret;

  if (driver == NULL || driver->get_table == NULL ||
      driver->target_index == NULL)
    {
      return -EINVAL;
    }

  if (g_cpufreq_policy != NULL)
    {
      return -EBUSY;
    }

  policy = kmm_zalloc(sizeof(*policy));
  if (policy == NULL)
    {
      return -ENOMEM;
    }

  policy->driver = driver;

  table = driver->get_table(policy);
  if (table == NULL)
    {
      kmm_free(policy);
      return -EINVAL;
    }

  for (count = 0; table[count].frequency != CPUFREQ_TABLE_END; count++)
    {
      if (count > 0 &&
          table[count].frequency <= table[count - 1].frequency)
        {
          syslog(LOG_ERR, "cpufreq: table must ascend\n");
          kmm_free(policy);
          return -EINVAL;
        }
    }

  if (count < 2)
    {
      syslog(LOG_ERR, "cpufreq: table too short to be worth scaling\n");
      kmm_free(policy);
      return -EINVAL;
    }

  policy->table    = table;
  policy->nentries = count;
  policy->current  = count - 1;   /* Assume fastest until told otherwise */
  nxmutex_init(&policy->lock);
  dq_init(&policy->requests);

  /* Prefer the lower half's reported frequency to the assumption above */

  if (driver->get_frequency != NULL)
    {
      ret = driver->get_frequency(policy);
      if (ret > 0)
        {
          unsigned int i;

          for (i = 0; i < count; i++)
            {
              if (table[i].frequency == (unsigned int)ret)
                {
                  policy->current = i;
                  break;
                }
            }
        }
    }

  g_cpufreq_policy = policy;

#ifdef CONFIG_CPUFREQ_CHARDEV
  ret = register_driver("/dev/cpufreq", &g_cpufreq_fops, 0666, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "cpufreq: cannot register /dev/cpufreq: %d\n", ret);

      /* The framework itself is still useful; carry on without it */
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: cpufreq_policy_get
 ****************************************************************************/

FAR struct cpufreq_policy *cpufreq_policy_get(void)
{
  return g_cpufreq_policy;
}

/****************************************************************************
 * Name: cpufreq_qos_add_request
 ****************************************************************************/

FAR struct cpufreq_qos *cpufreq_qos_add_request(
                             FAR struct cpufreq_policy *policy,
                             unsigned int min, unsigned int max)
{
  FAR struct cpufreq_qos *qos;

  if (policy == NULL)
    {
      return NULL;
    }

  qos = kmm_zalloc(sizeof(*qos));
  if (qos == NULL)
    {
      return NULL;
    }

  qos->min = min;
  qos->max = max;

  nxmutex_lock(&policy->lock);
  dq_addlast(&qos->node, &policy->requests);
  cpufreq_resolve(policy);
  nxmutex_unlock(&policy->lock);

  return qos;
}

/****************************************************************************
 * Name: cpufreq_qos_update_request
 ****************************************************************************/

int cpufreq_qos_update_request(FAR struct cpufreq_qos *qos,
                               unsigned int min, unsigned int max)
{
  FAR struct cpufreq_policy *policy = g_cpufreq_policy;
  int ret;

  if (qos == NULL || policy == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&policy->lock);
  qos->min = min;
  qos->max = max;
  ret = cpufreq_resolve(policy);
  nxmutex_unlock(&policy->lock);

  return ret;
}

/****************************************************************************
 * Name: cpufreq_qos_remove_request
 ****************************************************************************/

int cpufreq_qos_remove_request(FAR struct cpufreq_qos *qos)
{
  FAR struct cpufreq_policy *policy = g_cpufreq_policy;
  int ret;

  if (qos == NULL || policy == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&policy->lock);
  dq_rem(&qos->node, &policy->requests);
  ret = cpufreq_resolve(policy);
  nxmutex_unlock(&policy->lock);

  kmm_free(qos);
  return ret;
}

/****************************************************************************
 * Name: cpufreq_suspend
 ****************************************************************************/

int cpufreq_suspend(void)
{
  FAR struct cpufreq_policy *policy = g_cpufreq_policy;
  int ret = OK;

  if (policy == NULL)
    {
      return -ENODEV;
    }

  nxmutex_lock(&policy->lock);
  if (!policy->suspended)
    {
      if (policy->driver->suspend != NULL)
        {
          ret = policy->driver->suspend(policy);
        }

      if (ret >= 0)
        {
          policy->suspended = true;
        }
    }

  nxmutex_unlock(&policy->lock);
  return ret;
}

/****************************************************************************
 * Name: cpufreq_resume
 ****************************************************************************/

int cpufreq_resume(void)
{
  FAR struct cpufreq_policy *policy = g_cpufreq_policy;
  int ret = OK;

  if (policy == NULL)
    {
      return -ENODEV;
    }

  nxmutex_lock(&policy->lock);
  if (policy->suspended)
    {
      if (policy->driver->resume != NULL)
        {
          ret = policy->driver->resume(policy);
        }

      if (ret >= 0)
        {
          policy->suspended = false;

          /* Anything that changed while asleep applies now */

          cpufreq_resolve(policy);
        }
    }

  nxmutex_unlock(&policy->lock);
  return ret;
}
