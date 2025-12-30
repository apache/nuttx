/****************************************************************************
 * arch/arm64/src/common/arm64_pmuv3.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/perf.h>

#include <arm64_pmuv3.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware counters are supported at most */

#define ARM64PMU_MAX_HWEVENTS                32

/* The maximum value of the event ID */

#define ARM64PMU_MAX_COMMON_EVENTS           0x40

/* Event uses a 64bit counter */

#define ARM64PMU_EVT_64BIT                   0x00001

#define HW_OP_UNSUPPORTED                  0xFFFF
#define CACHE_OP_UNSUPPORTED               0xFFFF
#define C(_x)                              PERF_COUNT_HW_CACHE_##_x

#define PERF_MAP_ALL_UNSUPPORTED                                   \
        [0 ... PERF_COUNT_HW_MAX - 1] = HW_OP_UNSUPPORTED

#define PERF_CACHE_MAP_ALL_UNSUPPORTED                             \
        [0 ... C(MAX) - 1] = {                                     \
          [0 ... C(OP_MAX) - 1] = {                                \
            [0 ... C(RESULT_MAX) - 1] = CACHE_OP_UNSUPPORTED,      \
          },                                                       \
        }

#define to_arm64_pmu(p) (container_of(p, struct arm64_pmu_s, pmu))

#define per_cpu_ptr(ptr, cpu) ((typeof(*ptr)*) ((ptr) + (cpu)))

#define this_cpu_ptr(ptr) per_cpu_ptr(ptr, this_cpu())

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pmu_hw_events_s
{
  /* Pointer to the perf_event corresponding to the counter being used */

  struct perf_event_s *events[ARM64PMU_MAX_HWEVENTS];

  /* Each bit of bitmap represents whether a counter is being used */

  unsigned long       used_mask;

  /* PMU interrupt number */

  int                 irq;
};

struct arm64_pmu_s
{
  struct pmu_s  pmu;

  /* Each bit of the bitmap corresponds to a supported CPU */

  int           supported_cpus;

  /* PMU name */

  char          *name;

  /* PMU version */

  int           pmuver;
  int           (*handle_irq)(struct arm64_pmu_s *pmu);
  void          (*enable)(struct perf_event_s *event);
  void          (*disable)(struct perf_event_s *event);
  int           (*get_event_idx)(struct pmu_hw_events_s *hw_events,
                                 struct perf_event_s *event);
  void          (*clear_event_idx)(struct pmu_hw_events_s *hw_events,
                                   struct perf_event_s *event);
  int           (*set_event_filter)(struct hw_perf_event_s *evt,
                                    struct perf_event_attr_s *attr);
  uint64_t      (*read_counter)(struct perf_event_s *event);
  void          (*write_counter)(struct perf_event_s *event, uint64_t val);
  void          (*start)(struct arm64_pmu_s *);
  void          (*stop)(struct arm64_pmu_s *);
  void          (*reset)(void *);
  int           (*map_event)(struct perf_event_s *event);

  /* Number of event counters */

  int           num_events;

  /* Corresponds to common event n identification */

  uint64_t      pmceid_bitmap;

  /* Corresponds to common event (0x4000 + n) identification */

  uint64_t      pmceid_ext_bitmap;

  /* percpu hw_events */

  struct        pmu_hw_events_s *hw_events;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void arm64pmu_enable(FAR struct pmu_s *pmu);
static void arm64pmu_disable(FAR struct pmu_s *pmu);
static int arm64pmu_event_init(FAR struct perf_event_s *event);
static int arm64pmu_add(FAR struct perf_event_s *event, int flags);
static void arm64pmu_del(FAR struct perf_event_s *event, int flags);
static int arm64pmu_start(FAR struct perf_event_s *event, int flags);
static int arm64pmu_stop(FAR struct perf_event_s *event, int flags);
static int arm64pmu_read(FAR struct perf_event_s *event);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pmu_hw_events_s g_hw_events[CONFIG_SMP_NCPUS];

static struct pmu_ops_s g_arm64pmu_ops =
{
  .pmu_enable   = arm64pmu_enable,
  .pmu_disable  = arm64pmu_disable,
  .event_init   = arm64pmu_event_init,
  .event_add    = arm64pmu_add,
  .event_del    = arm64pmu_del,
  .event_start  = arm64pmu_start,
  .event_stop   = arm64pmu_stop,
  .event_read   = arm64pmu_read,
};

static const unsigned int pmuv3_perf_map[PERF_COUNT_HW_MAX] =
{
  PERF_MAP_ALL_UNSUPPORTED,
  [PERF_COUNT_HW_CPU_CYCLES]       = PMUV3_PERFCTR_CPU_CYCLES,
  [PERF_COUNT_HW_INSTRUCTIONS]     = PMUV3_PERFCTR_INST_RETIRED,
  [PERF_COUNT_HW_CACHE_REFERENCES] = PMUV3_PERFCTR_L1D_CACHE,
  [PERF_COUNT_HW_CACHE_MISSES]     = PMUV3_PERFCTR_L1D_CACHE_REFILL,
  [PERF_COUNT_HW_BRANCH_MISSES]    = PMUV3_PERFCTR_BR_MIS_PRED,
  [PERF_COUNT_HW_BUS_CYCLES]       = PMUV3_PERFCTR_BUS_CYCLES,
  [PERF_COUNT_HW_STALLED_CYCLES_FRONTEND]
                                   = PMUV3_PERFCTR_STALL_FRONTEND,
  [PERF_COUNT_HW_STALLED_CYCLES_BACKEND]
                                   = PMUV3_PERFCTR_STALL_BACKEND,
};

static const unsigned int pmuv3_perf_cache_map[PERF_COUNT_HW_CACHE_MAX]
                                  [PERF_COUNT_HW_CACHE_OP_MAX]
                                  [PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
  PERF_CACHE_MAP_ALL_UNSUPPORTED,

  [C(L1D)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_L1D_CACHE,
  [C(L1D)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_L1D_CACHE_REFILL,

  [C(L1I)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_L1I_CACHE,
  [C(L1I)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_L1I_CACHE_REFILL,

  [C(DTLB)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_L1D_TLB_REFILL,
  [C(DTLB)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_L1D_TLB,

  [C(ITLB)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_L1I_TLB_REFILL,
  [C(ITLB)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_L1I_TLB,

  [C(LL)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_LL_CACHE_MISS_RD,
  [C(LL)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_LL_CACHE_RD,

  [C(BPU)][C(OP_READ)][C(RESULT_ACCESS)]
                      = PMUV3_PERFCTR_BR_PRED,
  [C(BPU)][C(OP_READ)][C(RESULT_MISS)]
                      = PMUV3_PERFCTR_BR_MIS_PRED,
};

#ifdef CONFIG_ARCH_CORTEX_R82
static const unsigned armr82_perf_cache_map[PERF_COUNT_HW_CACHE_MAX]
                                           [PERF_COUNT_HW_CACHE_OP_MAX]
                                           [PERF_COUNT_HW_CACHE_RESULT_MAX] =
{
  PERF_CACHE_MAP_ALL_UNSUPPORTED,

  [C(L1D)][C(OP_PREFETCH)][C(RESULT_MISS)]
                          = ARMR82_IMPDEF_L1D_CACHE_REFILL_PREFETCH,

  [C(NODE)][C(OP_READ)][C(RESULT_ACCESS)]
                          = PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_RD,
  [C(NODE)][C(OP_WRITE)][C(RESULT_ACCESS)]
                          = PMUV3_IMPDEF_PERFCTR_BUS_ACCESS_WR,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool cpumask_test_cpu(int cpu, int cpumask)
{
  return (bool)((1U << cpu) & cpumask);
}

static inline bool test_and_set_bit(unsigned long nr,
                                    volatile unsigned long *addr)
{
  unsigned long mask = BIT_MASK(nr);
  unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
  unsigned long old = *p;

  *p = old | mask;
  return (old & mask) != 0;
}

static uint64_t arm64pmu_event_max_period(
                         FAR struct perf_event_s *event)
{
  if (event->hw.flags & ARM64PMU_EVT_64BIT)
    {
      return (uint64_t)UINT64_MAX;
    }
  else
    {
      return (uint64_t)UINT32_MAX;
    }
}

static int arm64pmu_map_cache_event(FAR const unsigned int (*cache_map)
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

static uint64_t arm64pmu_event_update(FAR struct perf_event_s *event)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  uint64_t max_period = arm64pmu_event_max_period(event);
  uint64_t delta;
  uint64_t new_raw_count;

  new_raw_count = arm64pmu->read_counter(event);

  delta = (new_raw_count) & max_period;

  atomic_fetch_add(&event->count, delta);

  return new_raw_count;
}

static int arm64pmu_map_hw_event(
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

static int arm64pmu_map_raw_event(uint32_t raw_event_mask, uint64_t config)
{
  return (int)(config & raw_event_mask);
}

static int arm64pmu_read(FAR struct perf_event_s *event)
{
  return arm64pmu_event_update(event);
}

static int arm64pmu_stop(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;

  if (!(hwc->state & PERF_HES_STOPPED))
    {
      arm64pmu->disable(event);
      arm64pmu_event_update(event);
      hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
    }

  return 0;
}

static int arm64pmu_start(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;

  hwc->state = 0;
  arm64pmu->enable(event);

  return 0;
}

static void arm64pmu_del(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(arm64pmu->hw_events);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int idx = hwc->idx;

  arm64pmu_stop(event, PERF_EF_UPDATE);
  hw_events->events[idx] = NULL;
  arm64pmu->clear_event_idx(hw_events, event);
  hwc->idx = -1;
}

static int arm64pmu_add(FAR struct perf_event_s *event, int flags)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(arm64pmu->hw_events);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int idx;

  if (!cpumask_test_cpu(this_cpu(), arm64pmu->supported_cpus))
    {
      return -ENOENT;
    }

  idx = arm64pmu->get_event_idx(hw_events, event);
  if (idx < 0)
    {
      return idx;
    }

  event->hw.idx = idx;
  arm64pmu->disable(event);
  hw_events->events[idx] = event;

  hwc->state = PERF_HES_STOPPED | PERF_HES_UPTODATE;
  if (flags & PERF_EF_START)
    {
      arm64pmu_start(event, PERF_EF_RELOAD);
    }

  return 0;
}

static int validate_group(FAR struct perf_event_s *event)
{
  return 0;
}

static int arm64pmu_event_init(FAR struct perf_event_s *event)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int mapping;

  if (event->cpu != -1 &&
      !cpumask_test_cpu(event->cpu, arm64pmu->supported_cpus))
    {
      return -ENOENT;
    }

  hwc->flags = 0;
  mapping = arm64pmu->map_event(event);

  if (mapping < 0)
    {
      return mapping;
    }

  hwc->idx          = -1;
  hwc->config_base  = 0;

  if (arm64pmu->set_event_filter &&
      arm64pmu->set_event_filter(hwc, &event->attr))
    {
      _err("ARM performance counters do not support\n");
      return -EOPNOTSUPP;
    }

  hwc->config_base |= (unsigned long)mapping;

  return validate_group(event);
}

static void arm64pmu_enable(FAR struct pmu_s *pmu)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(pmu);
  FAR struct pmu_hw_events_s *hw_events = this_cpu_ptr(arm64pmu->hw_events);

  if (!cpumask_test_cpu(this_cpu(), arm64pmu->supported_cpus))
    {
      return;
    }

  if (hw_events->used_mask)
    {
      arm64pmu->start(arm64pmu);
    }
}

static void arm64pmu_disable(FAR struct pmu_s *pmu)
{
  FAR struct arm64_pmu_s *arm64pmu = to_arm64_pmu(pmu);

  if (!cpumask_test_cpu(this_cpu(), arm64pmu->supported_cpus))
    return;

  arm64pmu->stop(arm64pmu);
}

static int arm64pmu_dispatch_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct arm64_pmu_s *arm64pmu = arg;

  return arm64pmu->handle_irq(arm64pmu);
}

static int arm64pmu_request_irq(int irq, FAR struct arm64_pmu_s *arm64pmu)
{
  int err = -EINVAL;
  const xcpt_t handler = arm64pmu_dispatch_irq;

  err = irq_attach(irq, handler, arm64pmu);
  if (err)
    {
      _err("ERROR: arm pmu irq attach failed!\n");
      return err;
    }

  up_enable_irq(irq);

  return OK;
}

static int arm64pmu_request_irqs(FAR struct arm64_pmu_s *arm64pmu)
{
  FAR struct pmu_hw_events_s *hw_events = arm64pmu->hw_events;
  int err;

  err = arm64pmu_request_irq(hw_events->irq, arm64pmu);

  return err;
}

static FAR struct arm64_pmu_s *arm64pmu_alloc(void)
{
  FAR struct arm64_pmu_s *arm64pmu;

  arm64pmu = kmm_zalloc(sizeof(struct arm64_pmu_s));
  if (!arm64pmu)
    {
      return NULL;
    }

  arm64pmu->hw_events = g_hw_events;
  arm64pmu->pmu.ops = &g_arm64pmu_ops;

  return arm64pmu;
}

static void arm64pmu_free(FAR struct arm64_pmu_s *arm64pmu)
{
  kmm_free(arm64pmu);
}

static int arm64pmu_register(FAR struct arm64_pmu_s *arm64pmu)
{
  return perf_pmu_register(&arm64pmu->pmu, arm64pmu->name);
}

static int arm64pmu_map_event(FAR struct perf_event_s *event,
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
        return arm64pmu_map_hw_event(event_map, config);
      case PERF_TYPE_HW_CACHE:
        return arm64pmu_map_cache_event(cache_map, config);
      case PERF_TYPE_RAW:
        return arm64pmu_map_raw_event(raw_event_mask, config);
    }

  return -ENOENT;
}

static inline bool pmuv3_event_is_64bit(FAR struct perf_event_s *event)
{
  return (bool)(event->attr.config1 & 0x1);
}

static inline void pmuv3_pmcr_write(uint32_t val)
{
  val &= PMU_PMCR_MASK;
  arm64_isb();
  write_pmcr(val);
}

static inline int pmuv3_counter_has_overflowed(uint32_t pmovsr, int idx)
{
  return pmovsr & BIT(PMU_IDX_TO_COUNTER(idx));
}

static inline void pmuv3_write_event_type(
                     FAR struct perf_event_s *event)
{
  FAR struct hw_perf_event_s *hwc = &event->hw;
  int idx = hwc->idx;

  if (idx == PMU_IDX_CYCLE_COUNTER)
    {
      write_pmccfiltr(hwc->config_base);
    }
  else
    {
      write_pmevtypern(PMU_IDX_TO_COUNTER(idx),
                       hwc->config_base & PMU_EVTYPE_MASK);
    }
}

static inline void pmuv3_enable_event_counter(
                     FAR struct perf_event_s *event)
{
  uint32_t mask = BIT(PMU_IDX_TO_COUNTER(event->hw.idx));

  arm64_isb();
  write_pmcntenset(mask);
}

static inline void pmuv3_disable_counter(uint32_t mask)
{
  write_pmcntenclr(mask);
  arm64_isb();
}

static inline void pmuv3_disable_event_counter(
                     FAR struct perf_event_s *event)
{
  uint32_t mask = BIT(PMU_IDX_TO_COUNTER(event->hw.idx));

  pmuv3_disable_counter(mask);
}

static inline void pmuv3_enable_event_irq(
                     FAR struct perf_event_s *event)
{
  uint32_t counter = PMU_IDX_TO_COUNTER(event->hw.idx);
  write_pmintenset(BIT(counter));
}

static inline void pmuv3_disable_intens(uint32_t mask)
{
  write_pmintenclr(mask);
  arm64_isb();
  write_pmovsclr(mask);
  arm64_isb();
}

static inline void pmuv3_disable_event_irq(
                     FAR struct perf_event_s *event)
{
  uint32_t counter = PMU_IDX_TO_COUNTER(event->hw.idx);
  pmuv3_disable_intens(BIT(counter));
}

static inline uint32_t pmuv3_getreset_flags(void)
{
  uint32_t value;

  value = read_pmovsclr();
  write_pmovsclr(value);

  return value;
}

static void pmuv3_enable_user_access(FAR struct arm64_pmu_s *cpu_pmu)
{
  int i;
  FAR struct pmu_hw_events_s *cpuc = this_cpu_ptr(cpu_pmu->hw_events);

  for (i = 0; i < cpu_pmu->num_events; i++)
    {
      if (!test_bit(i, &cpuc->used_mask))
        {
          continue;
        }

      if (i == PMU_IDX_CYCLE_COUNTER)
        {
          write_pmccntr(0);
        }
      else
        {
          write_pmevcntrn(PMU_IDX_TO_COUNTER(i), 0);
        }
    }

  write_pmuserenr(0);
  write_pmuserenr(PMU_USERENR_ER | PMU_USERENR_CR);
}

static void pmuv3_enable_event(FAR struct perf_event_s *event)
{
  pmuv3_disable_event_counter(event);
  pmuv3_write_event_type(event);
  pmuv3_enable_event_irq(event);
  pmuv3_enable_event_counter(event);
}

static void pmuv3_disable_event(FAR struct perf_event_s *event)
{
  pmuv3_disable_event_counter(event);
  pmuv3_disable_event_irq(event);
}

static uint64_t pmuv3_read_counter(FAR struct perf_event_s *event)
{
  int idx = event->hw.idx;
  uint64_t value;

  if (idx == PMU_IDX_CYCLE_COUNTER)
    value = read_pmccntr();
  else
    value = read_pmevcntrn(PMU_IDX_TO_COUNTER(idx));

  return value;
}

static void pmuv3_write_counter(FAR struct perf_event_s *event,
                                   uint64_t value)
{
  int idx = event->hw.idx;

  if (idx == PMU_IDX_CYCLE_COUNTER)
    write_pmccntr(value);
  else
    write_pmevcntrn(PMU_IDX_TO_COUNTER(idx), value);
}

static void pmuv3_start(FAR struct arm64_pmu_s *cpu_pmu)
{
  pmuv3_enable_user_access(cpu_pmu);

  pmuv3_pmcr_write(read_pmcr() | PMU_PMCR_E);
}

static void pmuv3_stop(FAR struct arm64_pmu_s *cpu_pmu)
{
  pmuv3_pmcr_write(read_pmcr() & ~PMU_PMCR_E);
}

static int pmuv3_handle_irq(FAR struct arm64_pmu_s *cpu_pmu)
{
  uint32_t pmovsr;
  FAR struct pmu_hw_events_s *cpuc = this_cpu_ptr(cpu_pmu->hw_events);
  int idx;

  /* Get the IRQ flags and reset */

  pmovsr = pmuv3_getreset_flags();

  /* Return directly if there is no overflow */

  if (!pmovsr)
    {
      return 0;
    }

  /* Stop the PMU while processing the counter overflows */

  pmuv3_stop(cpu_pmu);

  /* Iterate over the overflow counter and update the data */

  for (idx = 0; idx < cpu_pmu->num_events; ++idx)
    {
      struct perf_event_s *event = cpuc->events[idx];

      if (!event)
        {
          continue;
        }

      if (!pmuv3_counter_has_overflowed(pmovsr, idx))
        {
          continue;
        }

      /* Update data */

      arm64pmu_event_update(event);

      if (perf_event_overflow(event))
        {
          cpu_pmu->disable(event);
        }
    }

  /* Start the PMU after the interrupt processing is complete */

  pmuv3_start(cpu_pmu);

  return 1;
}

static int pmuv3_get_event_idx(FAR struct pmu_hw_events_s *cpuc,
                                  FAR struct perf_event_s *event)
{
  FAR struct arm64_pmu_s *cpu_pmu = to_arm64_pmu(event->pmu);
  FAR struct hw_perf_event_s *hwc = &event->hw;
  unsigned long evtype = hwc->config_base & PMU_EVTYPE_EVENT;
  int idx;

  if (evtype == PMUV3_PERFCTR_CPU_CYCLES)
    {
      if (!test_and_set_bit(PMU_IDX_CYCLE_COUNTER, &cpuc->used_mask))
        {
          return PMU_IDX_CYCLE_COUNTER;
        }
    }

  for (idx = PMU_IDX_COUNTER0; idx < cpu_pmu->num_events; idx++)
    {
      if (!test_and_set_bit(idx, &cpuc->used_mask))
        {
          return idx;
        }
    }

  return -EAGAIN;
}

static void pmuv3_clear_event_idx(FAR struct pmu_hw_events_s *cpuc,
                                     FAR struct perf_event_s *event)
{
  __clear_bit(event->hw.idx, &cpuc->used_mask);
}

static int pmuv3_event_idx(FAR struct perf_event_s *event)
{
  if (event->hw.idx == PMU_IDX_CYCLE_COUNTER)
    {
      return PMU_IDX_CYCLE_COUNTER_USER;
    }

  return event->hw.idx;
}

static int pmuv3_set_event_filter(FAR struct hw_perf_event_s *event,
                                     FAR struct perf_event_attr_s *attr)
{
  unsigned long config_base = 0;

  if (attr->exclude_idle)
    {
      return -EPERM;
    }

  if (attr->exclude_kernel)
    {
      config_base |= PMU_EXCLUDE_EL1;
    }

  if (attr->exclude_user)
    {
      config_base |= PMU_EXCLUDE_EL0;
    }

  event->config_base = config_base;

  return 0;
}

static void pmuv3_reset(FAR void *info)
{
  uint32_t pmcr;

  pmuv3_disable_counter(UINT32_MAX);
  pmuv3_disable_intens(UINT32_MAX);

  pmcr = PMU_PMCR_P | PMU_PMCR_C | PMU_PMCR_LC;

  pmuv3_pmcr_write(pmcr);
}

static int pmuv3_map_event_id(FAR struct arm64_pmu_s *armpmu,
                                    FAR struct perf_event_s *event)
{
  if (event->attr.type == PERF_TYPE_HARDWARE &&
      event->attr.config == PERF_COUNT_HW_BRANCH_INSTRUCTIONS)
    {
      if (test_bit(PMUV3_PERFCTR_PC_WRITE_RETIRED,
                   &armpmu->pmceid_bitmap))
        {
          return PMUV3_PERFCTR_PC_WRITE_RETIRED;
        }

      if (test_bit(PMUV3_PERFCTR_BR_RETIRED,
                   &armpmu->pmceid_bitmap))
        {
          return PMUV3_PERFCTR_BR_RETIRED;
        }

      return HW_OP_UNSUPPORTED;
    }

  return arm64pmu_map_event(event, &pmuv3_perf_map,
                          &pmuv3_perf_cache_map,
                          PMU_EVTYPE_EVENT);
}

static int pmuv3_map_extra_event(FAR struct perf_event_s *event,
                                 FAR const unsigned int (*extra_event_map)
                                           [PERF_COUNT_HW_MAX],
                                 FAR const unsigned int (*extra_cache_map)
                                           [PERF_COUNT_HW_CACHE_MAX]
                                           [PERF_COUNT_HW_CACHE_OP_MAX]
                                           [PERF_COUNT_HW_CACHE_RESULT_MAX])
{
  int hw_event_id;
  FAR struct arm64_pmu_s *armpmu = to_arm64_pmu(event->pmu);

  hw_event_id = pmuv3_map_event_id(armpmu, event);

  if (hw_event_id == PMUV3_PERFCTR_CHAIN)
    {
      return -EINVAL;
    }

  if (pmuv3_event_is_64bit(event))
    {
      event->hw.flags |= ARM64PMU_EVT_64BIT;
    }

  event->hw.flags |= PERF_EVENT_FLAG_USER_READ_CNT;

  if ((hw_event_id > 0) && (hw_event_id < ARM64PMU_MAX_COMMON_EVENTS)
       && test_bit(hw_event_id, &armpmu->pmceid_bitmap))
    {
      return hw_event_id;
    }

  return arm64pmu_map_event(event, extra_event_map, extra_cache_map,
                          PMU_EVTYPE_EVENT);
}

#ifdef CONFIG_ARCH_CORTEX_R82
static int armr82_map_event(FAR struct perf_event_s *event)
{
  return pmuv3_map_extra_event(event, NULL, &armr82_perf_cache_map);
}
#else
static int pmuv3_map_event(FAR struct perf_event_s *event)
{
  return pmuv3_map_extra_event(event, NULL, NULL);
}
#endif

static int pmuv3_common_pmu_init(FAR struct arm64_pmu_s *cpu_pmu, char *name,
                          FAR int (*map_event)(struct perf_event_s *event))
{
  uint64_t pmceid[2];
  int pmuver;
  int cpu;

  pmuver = read_pmuver();
  if (!pmuv3_implemented(pmuver))
    {
      return -1;
    }

  cpu_pmu->pmuver = pmuver;

  cpu_pmu->num_events = (read_pmcr() >> PMU_PMCR_N_SHIFT)
                         & PMU_PMCR_N_MASK;
  cpu_pmu->num_events += 1;

  pmceid[0] = read_pmceid0();
  pmceid[1] = read_pmceid1();
  cpu_pmu->pmceid_bitmap = (pmceid[0] & 0xffffffff) | (pmceid[1] << 32);
  cpu_pmu->pmceid_ext_bitmap = (pmceid[0] >> 32) |
                               (pmceid[1] & 0xffffffff00000000);

  cpu_pmu->handle_irq         = pmuv3_handle_irq;
  cpu_pmu->enable             = pmuv3_enable_event;
  cpu_pmu->disable            = pmuv3_disable_event;
  cpu_pmu->read_counter       = pmuv3_read_counter;
  cpu_pmu->write_counter      = pmuv3_write_counter;
  cpu_pmu->get_event_idx      = pmuv3_get_event_idx;
  cpu_pmu->clear_event_idx    = pmuv3_clear_event_idx;
  cpu_pmu->start              = pmuv3_start;
  cpu_pmu->stop               = pmuv3_stop;
  cpu_pmu->reset              = pmuv3_reset;
  cpu_pmu->set_event_filter   = pmuv3_set_event_filter;
  cpu_pmu->pmu.ops->event_idx = pmuv3_event_idx;
  cpu_pmu->name               = name;
  cpu_pmu->map_event          = map_event;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      struct pmu_hw_events_s *events;

      cpu_pmu->supported_cpus |= (1U << cpu);
      events = per_cpu_ptr(cpu_pmu->hw_events, cpu);
      events->irq = 23;
    }

  return 0;
}

static int pmuv3_pmu_init(FAR struct arm64_pmu_s *cpu_pmu)
{
#ifdef CONFIG_ARCH_CORTEX_R82
  return pmuv3_common_pmu_init(cpu_pmu, "arm,cortex-r82-pmu",
                               armr82_map_event);
#else
  return pmuv3_common_pmu_init(cpu_pmu, "arm,arm-pmuv3", pmuv3_map_event);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int arm64_pmu_initialize(void)
{
  FAR struct arm64_pmu_s *arm64pmu;
  int ret = OK;

  arm64pmu = arm64pmu_alloc();
  if (!arm64pmu)
    {
      _err("ERROR: alloc arm_pmu failed!\n");
      return -ENOMEM;
    }

  ret = pmuv3_pmu_init(arm64pmu);
  if (ret)
    {
      _err("ERROR: init func failed!\n");
      goto out_free;
    }

  ret = arm64pmu_request_irqs(arm64pmu);
  if (ret)
    {
      goto out_free;
    }

  ret = arm64pmu_register(arm64pmu);
  if (ret)
    {
      _err("ERROR: failed to register PMU devices!\n");
      goto out_free;
    }

  return OK;

out_free:
  arm64pmu_free(arm64pmu);
  return ret;
}
