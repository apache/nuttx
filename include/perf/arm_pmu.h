/****************************************************************************
 * include/perf/arm_pmu.h
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

#ifndef __INCLUDE_PERF_ARM_PMU_H
#define __INCLUDE_PERF_ARM_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/spinlock.h>
#include <nuttx/list.h>
#include <nuttx/nuttx.h>
#include <nuttx/perf.h>
#include <nuttx/bits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Hardware counters are supported at most */

#define ARMPMU_MAX_HWEVENTS                32

/* The maximum value of the event ID */

#define ARMPMU_MAX_COMMON_EVENTS           0x40

/* Event uses a 64bit counter */

#define ARMPMU_EVT_64BIT                   0x00001

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

#define to_arm_pmu(p) (container_of(p, struct arm_pmu_s, pmu))

#define per_cpu_ptr(ptr, cpu) ((typeof(*ptr)*) ((ptr) + (cpu)))

#define this_cpu_ptr(ptr) per_cpu_ptr(ptr, up_cpu_index())

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pmu_hw_events_s
{
  /* Pointer to the perf_event corresponding to the counter being used */

  struct perf_event_s *events[ARMPMU_MAX_HWEVENTS];

  /* Each bit of bitmap represents whether a counter is being used */

  unsigned long       used_mask;

  /* PMU interrupt number */

  int                 irq;
};

struct arm_pmu_s
{
  struct pmu_s  pmu;

  /* Each bit of the bitmap corresponds to a supported CPU */

  int           supported_cpus;

  /* PMU name */

  char          *name;

  /* PMU version */

  int           pmuver;
  int           (*handle_irq)(struct arm_pmu_s *pmu);
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
  void          (*start)(struct arm_pmu_s *);
  void          (*stop)(struct arm_pmu_s *);
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

typedef int (*armpmu_init_fn)(struct arm_pmu_s *);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline bool cpumask_test_cpu(int cpu, int cpumask)
{
  return (bool)((1U << cpu) & cpumask);
}

static inline bool test_and_set_bit(unsigned long nr,
                                    volatile unsigned long *addr)
{
  unsigned long mask = BIT_WORD_MASK(nr);
  unsigned long *p = ((unsigned long *)addr) + BIT_WORD(nr);
  unsigned long old = *p;

  *p = old | mask;
  return (old & mask) != 0;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint64_t armpmu_event_update(struct perf_event_s *event);
int armpmu_driver_init(FAR void *fn);
int armpmu_map_event(struct perf_event_s *event,
                     const unsigned (*event_map)[PERF_COUNT_HW_MAX],
                     const unsigned (*cache_map)
                                    [PERF_COUNT_HW_CACHE_MAX]
                                    [PERF_COUNT_HW_CACHE_OP_MAX]
                                    [PERF_COUNT_HW_CACHE_RESULT_MAX],
                     uint32_t raw_event_mask);

#endif /* __INCLUDE_PERF_ARM_PMU_H */
