/****************************************************************************
 * drivers/devfreq/devfreq_ondemand.c
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
#include <nuttx/devfreq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVFREQ_MIN_SAMPLING_INTERVAL   (2 * USEC_PER_TICK)
#define DEVFREQ_LOAD_THRESHOLD_MAX      (100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct devfreq_ondemand_s
{
  struct work_s work;
  uint32_t threshold;
  uint32_t sample_rate;
  uint32_t target_freq;
  FAR struct qos_request_s *req;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int devfreq_gov_ondemand_init(FAR struct devfreq_s *dev);
static int devfreq_gov_ondemand_exit(FAR struct devfreq_s *dev);
static int devfreq_gov_ondemand_start(FAR struct devfreq_s *dev);
static void devfreq_gov_ondemand_stop(FAR struct devfreq_s *dev);
static uint32_t devfreq_gov_ondemand_limit(FAR struct devfreq_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct devfreq_governor_s g_devfreq_gov_ondemand =
{
  .name   = "ondemand",
  .init   = devfreq_gov_ondemand_init,
  .exit   = devfreq_gov_ondemand_exit,
  .start  = devfreq_gov_ondemand_start,
  .stop   = devfreq_gov_ondemand_stop,
  .limit  = devfreq_gov_ondemand_limit,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t devfreq_gov_ondemand_cpuload(void)
{
  struct cpuload_s loadavg;
  uint32_t idleload = 0;
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      clock_cpuload(cpu, &loadavg);
      idleload += loadavg.active * 100 / loadavg.total;
    }

  return 100 - idleload;
}

static void devfreq_ondemand_worker(FAR void *arg)
{
  FAR struct devfreq_s *dev = arg;
  FAR struct devfreq_ondemand_s *data = dev->governor_data;
  uint32_t cpuload;

  cpuload = devfreq_gov_ondemand_cpuload();
  nxmutex_lock(&dev->lock);
  if (cpuload > CONFIG_DEVFREQ_LOAD_THRESHOLD)
    {
      if (dev->cur < dev->max)
        {
          data->target_freq = dev->max;
        }
      else
        {
          data->target_freq = dev->cur;
        }
    }
  else
    {
      data->target_freq = dev->min + cpuload *
                          (dev->max - dev->min) / 100;
    }

  nxmutex_unlock(&dev->lock);

  devfreq_qos_update_request(dev, data->req, dev->min, dev->max);
  work_queue(HPWORK,
             &data->work,
             devfreq_ondemand_worker,
             dev,
             data->sample_rate / USEC_PER_TICK);
}

static int devfreq_gov_ondemand_init(FAR struct devfreq_s *dev)
{
  FAR struct devfreq_ondemand_s *data;

  data = kmm_zalloc(sizeof(struct devfreq_ondemand_s));
  if (!data)
    {
      return -ENOMEM;
    }

  data->req = devfreq_qos_add_request(dev, dev->min, dev->max);
  data->threshold = MIN(DEVFREQ_LOAD_THRESHOLD_MAX,
                        CONFIG_DEVFREQ_LOAD_THRESHOLD);
  data->sample_rate = MAX(DEVFREQ_MIN_SAMPLING_INTERVAL,
                          CONFIG_DEVFREQ_SAMPLE_RATE);
  dev->governor_data = data;
  return 0;
}

static int devfreq_gov_ondemand_exit(FAR struct devfreq_s *dev)
{
  FAR struct devfreq_ondemand_s *data = dev->governor_data;

  devfreq_qos_remove_request(dev, data->req);

  kmm_free(data);
  return 0;
}

static int devfreq_gov_ondemand_start(FAR struct devfreq_s *dev)
{
  FAR struct devfreq_ondemand_s *data = dev->governor_data;

  work_queue(HPWORK,
             &data->work,
             devfreq_ondemand_worker,
             dev,
             0);
  return 0;
}

static void devfreq_gov_ondemand_stop(FAR struct devfreq_s *dev)
{
  FAR struct devfreq_ondemand_s *data = dev->governor_data;

  if (sched_idletask())
    {
      work_cancel(HPWORK, &data->work);
    }
  else
    {
      work_cancel_sync(HPWORK, &data->work);
    }
}

static uint32_t devfreq_gov_ondemand_limit(FAR struct devfreq_s *dev)
{
  FAR struct devfreq_ondemand_s *data = dev->governor_data;
  uint32_t freq = data->target_freq;

  if (freq > dev->max)
    {
      freq = dev->max;
    }

  if (freq < dev->min)
    {
      freq = dev->min;
    }

  return freq;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct devfreq_governor_s *devfreq_ondemand(void)
{
  return &g_devfreq_gov_ondemand;
}
