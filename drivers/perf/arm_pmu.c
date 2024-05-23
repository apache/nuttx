/****************************************************************************
 * drivers/perf/arm_pmu.c
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
#include <stdatomic.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/perf.h>

#include <perf/arm_pmu.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void armpmu_enable(FAR struct pmu_s *pmu);
static void armpmu_disable(FAR struct pmu_s *pmu);
static int armpmu_event_init(FAR struct perf_event_s *event);
static int armpmu_add(FAR struct perf_event_s *event, int flags);
static void armpmu_del(FAR struct perf_event_s *event, int flags);
static int armpmu_start(FAR struct perf_event_s *event, int flags);
static int armpmu_stop(FAR struct perf_event_s *event, int flags);
static int armpmu_read(FAR struct perf_event_s *event);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pmu_hw_events_s g_hw_events[CONFIG_SMP_NCPUS];

static struct pmu_ops_s g_armpmu_ops =
{
  .pmu_enable   = armpmu_enable,
  .pmu_disable  = armpmu_disable,
  .event_init   = armpmu_event_init,
  .event_add    = armpmu_add,
  .event_del    = armpmu_del,
  .event_start  = armpmu_start,
  .event_stop   = armpmu_stop,
  .event_read   = armpmu_read,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t armpmu_event_max_period(
                         FAR struct perf_event_s *event)
{
  if (event->hw.flags & ARMPMU_EVT_64BIT)
    {
      return (uint64_t)UINT64_MAX;
    }
  else
    {
      return (uint64_t)UINT32_MAX;
    }
}

static int armpmu_map_cache_event(FAR const unsigned int (*cache_map)
                                        [PERF_COUNT_HW_CACHE_MAX]
                                        [PERF_COUNT_HW_CACHE_OP_MAX]
                                        [PERF_COUNT_HW_CACHE_RESULT_MAX],
                                  uint64_t config)
{
  unsigned int cache_type;
  unsigned int cache_op;
  unsigned int cache_result;
  unsigned int mapping;

  cache_type = (config >>  0) & 0xff;
  if (cache_type >= PERF_COUNT_HW_CACHE_MAX)
    {
      return -EINVAL;
    }

  cache_op = (config >>  8) & 0xff;
  if (cache_op >= PERF_COUNT_HW_CACHE_OP_MAX)
    {
      return -EINVAL;
    }

  cache_result = (config >> 16) & 0xff;
  if (cache_result >= PERF_COUNT_HW_CACHE_RESULT_MAX)
    {
      return -EINVAL;
    }

  if (!cache_map)
    {
      return -ENOENT;
    }

  mapping = (*cache_map)[cache_type][cache_op][cache_result];

  if (mapping == CACHE_OP_UNSUPPORTED)
    {
      return -ENOENT;
    }

  return mapping;
}

static int armpmu_map_hw_event(
                FAR const unsigned int (*event_map)[PERF_COUNT_HW_MAX],
                uint64_t config)
{
  int mapping;

  if (config >= PERF_COUNT_HW_MAX)
    {
      return -EINVAL;
    }

  if (!event_map)
    {
      return -ENOENT;
    }

  mapping = (*event_map)[config];

  if (mapping == HW_OP_UNSUPPORTED)
    {
      return -ENOENT;
    }

  return mapping;
}

static int armpmu_map_raw_event(uint32_t raw_event_mask, uint64_t config)
{
  return (int)(config & raw_event_mask);
}

static int armpmu_read(FAR struct perf_event_s *event)
{
  return armpmu_event_update(event);
}

static int armpmu_stop(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;

  if (!(hwc->state & PERF_HES_STOPPED))
    {
      armpmu->disable(event);
      armpmu_event_update(event);
      hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
    }

  return 0;
}

static int armpmu_start(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;

  hwc->state = 0;
  armpmu->enable(event);

  return 0;
}

static void armpmu_del(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(armpmu->hw_events);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int idx = hwc->idx;

  armpmu_stop(event, PERF_EF_UPDATE);
  hw_events->events[idx] = NULL;
  armpmu->clear_event_idx(hw_events, event);
  hwc->idx = -1;
}

static int armpmu_add(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(armpmu->hw_events);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int idx;

  if (!cpumask_test_cpu(up_cpu_index(), armpmu->supported_cpus))
    {
      return -ENOENT;
    }

  idx = armpmu->get_event_idx(hw_events, event);
  if (idx < 0)
    {
      return idx;
    }

  event->hw.idx = idx;
  armpmu->disable(event);
  hw_events->events[idx] = event;

  hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
  if (flags & PERF_EF_START)
    {
      armpmu_start(event, PERF_EF_RELOAD);
    }

  return 0;
}

static int validate_group(FAR struct perf_event_s *event)
{
  return 0;
}

static int armpmu_event_init(FAR struct perf_event_s *event)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int mapping;

  if (event->cpu != -1 &&
      !cpumask_test_cpu(event->cpu, armpmu->supported_cpus))
    {
      return -ENOENT;
    }

  hwc->flags = 0;
  mapping = armpmu->map_event(event);

  if (mapping < 0)
    {
      return mapping;
    }

  hwc->idx          = -1;
  hwc->config_base  = 0;

  if (armpmu->set_event_filter &&
      armpmu->set_event_filter(hwc, &event->attr))
    {
      _err("ARM performance counters do not support\n");
      return -EOPNOTSUPP;
    }

  hwc->config_base |= (unsigned long)mapping;

  return validate_group(event);
}

static void armpmu_enable(FAR struct pmu_s *pmu)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(armpmu->hw_events);

  if (!cpumask_test_cpu(up_cpu_index(), armpmu->supported_cpus))
    {
      return;
    }

  if (hw_events->used_mask)
    {
      armpmu->start(armpmu);
    }
}

static void armpmu_disable(FAR struct pmu_s *pmu)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(pmu);

  if (!cpumask_test_cpu(up_cpu_index(), armpmu->supported_cpus))
    return;

  armpmu->stop(armpmu);
}

static int armpmu_dispatch_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct arm_pmu_s *armpmu = arg;

  return armpmu->handle_irq(armpmu);
}

static int armpmu_request_irq(int irq, FAR struct arm_pmu_s *armpmu)
{
  int err = -EINVAL;
  const xcpt_t handler = armpmu_dispatch_irq;

  err = irq_attach(irq, handler, armpmu);
  if (err)
    {
      _err("ERROR: arm pmu irq attach failed!\n");
      return err;
    }

  up_enable_irq(irq);

  return OK;
}

static int armpmu_request_irqs(FAR struct arm_pmu_s *armpmu)
{
  FAR struct pmu_hw_events_s *hw_events = armpmu->hw_events;
  int err;

  err = armpmu_request_irq(hw_events->irq, armpmu);

  return err;
}

static FAR struct arm_pmu_s *armpmu_alloc(void)
{
  FAR struct arm_pmu_s *armpmu;

  armpmu = kmm_zalloc(sizeof(struct arm_pmu_s));
  if (!armpmu)
    {
      return NULL;
    }

  armpmu->hw_events = g_hw_events;
  armpmu->pmu.ops = &g_armpmu_ops;

  return armpmu;
}

static void armpmu_free(FAR struct arm_pmu_s *armpmu)
{
  kmm_free(armpmu);
}

static int armpmu_register(FAR struct arm_pmu_s *armpmu)
{
  return perf_pmu_register(&armpmu->pmu, armpmu->name, PERF_TYPE_HARDWARE);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int armpmu_map_event(FAR struct perf_event_s *event,
                     FAR const unsigned int (*event_map)[PERF_COUNT_HW_MAX],
                     FAR const unsigned int (*cache_map)
                                    [PERF_COUNT_HW_CACHE_MAX]
                                    [PERF_COUNT_HW_CACHE_OP_MAX]
                                    [PERF_COUNT_HW_CACHE_RESULT_MAX],
                     uint32_t raw_event_mask)
{
  uint64_t config = event->attr.config;
  int type = event->attr.type;

  switch (type)
    {
      case PERF_TYPE_HARDWARE:
        return armpmu_map_hw_event(event_map, config);
      case PERF_TYPE_HW_CACHE:
        return armpmu_map_cache_event(cache_map, config);
      case PERF_TYPE_RAW:
        return armpmu_map_raw_event(raw_event_mask, config);
    }

  return -ENOENT;
}

uint64_t armpmu_event_update(FAR struct perf_event_s *event)
{
  FAR struct arm_pmu_s *armpmu = to_arm_pmu(event->pmu);
  uint64_t max_period = armpmu_event_max_period(event);
  uint64_t delta;
  uint64_t new_raw_count;

  new_raw_count = armpmu->read_counter(event);

  delta = (new_raw_count) & max_period;

  atomic_fetch_add(&event->count, delta);

  return new_raw_count;
}

int armpmu_driver_init(FAR void *fn)
{
  FAR armpmu_init_fn init_fn = (armpmu_init_fn)fn;
  FAR struct arm_pmu_s *armpmu;
  int ret = -1;

  armpmu = armpmu_alloc();
  if (!armpmu)
    {
      _err("ERROR: alloc arm_pmu failed!\n");
    }

  if (init_fn)
    {
      ret = init_fn(armpmu);
      if (ret)
        {
          _err("ERROR: init func failed!\n");
          goto out_free;
        }
    }

  ret = armpmu_request_irqs(armpmu);
  if (ret)
    {
      goto out_free;
    }

  ret = armpmu_register(armpmu);
  if (ret)
    {
      _err("ERROR: failed to register PMU devices!\n");
      goto out_free;
    }

  return 0;

out_free:
  armpmu_free(armpmu);
  return ret;
}
