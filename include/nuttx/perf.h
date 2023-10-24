/****************************************************************************
 * include/nuttx/perf.h
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

#ifndef __INCLUDE_NUTTX_PERF_H
#define __INCLUDE_NUTTX_PERF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PERF_EVENT_IOC_ENABLE          _PERFIOC(0x0000)
#define PERF_EVENT_IOC_DISABLE         _PERFIOC(0x0001)
#define PERF_EVENT_IOC_REFRESH         _PERFIOC(0x0002)
#define PERF_EVENT_IOC_RESET           _PERFIOC(0x0003)
#define PERF_EVENT_IOC_PERIOD          _PERFIOC(0x0004)
#define PERF_EVENT_IOC_SET_OUTPUT      _PERFIOC(0x0005)
#define PERF_EVENT_IOC_SET_FILTER      _PERFIOC(0x0006)
#define PERF_EVENT_IOC_ID              _PERFIOC(0x0007)

#define PERF_EF_START                  0x01
#define PERF_EF_RELOAD                 0x02
#define PERF_EF_UPDATE                 0x04

/* hw_perf_event_s::state; used to track the PERF_EF_* state. */

#define PERF_HES_STOPPED               0x01
#define PERF_HES_UPTODATE              0x02
#define PERF_HES_ARCH                  0x04

#define PERF_ATTACH_CONTEXT            0x01
#define PERF_ATTACH_GROUP              0x02
#define PERF_ATTACH_TASK               0x04
#define PERF_ATTACH_TASK_DATA          0x08
#define PERF_ATTACH_ITRACE             0x10
#define PERF_ATTACH_SCHED_CB           0x20
#define PERF_ATTACH_CHILD              0x40

#define PERF_IOC_FLAG_GROUP            1

#define PERF_EVENT_FLAG_ARCH           0x000fffff
#define PERF_EVENT_FLAG_USER_READ_CNT  0x80000000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* attr.type */

enum perf_type_id_e
{
  PERF_TYPE_HARDWARE   = 0,
  PERF_TYPE_SOFTWARE   = 1,
  PERF_TYPE_TRACEPOINT = 2,
  PERF_TYPE_HW_CACHE   = 3,
  PERF_TYPE_RAW        = 4,
  PERF_TYPE_BREAKPOINT = 5,
  PERF_TYPE_MAX,
};

/* attr.config for PERF_TYPE_HARDWARE */

enum perf_hw_id_e
{
  PERF_COUNT_HW_CPU_CYCLES              = 0,
  PERF_COUNT_HW_INSTRUCTIONS            = 1,
  PERF_COUNT_HW_CACHE_REFERENCES        = 2,
  PERF_COUNT_HW_CACHE_MISSES            = 3,
  PERF_COUNT_HW_BRANCH_INSTRUCTIONS     = 4,
  PERF_COUNT_HW_BRANCH_MISSES           = 5,
  PERF_COUNT_HW_BUS_CYCLES              = 6,
  PERF_COUNT_HW_STALLED_CYCLES_FRONTEND = 7,
  PERF_COUNT_HW_STALLED_CYCLES_BACKEND  = 8,
  PERF_COUNT_HW_REF_CPU_CYCLES          = 9,
  PERF_COUNT_HW_MAX,
};

/* attr.config for PERF_TYPE_SOFTWARE */

enum perf_sw_ids_e
{
  PERF_COUNT_SW_CPU_CLOCK        = 0,
  PERF_COUNT_SW_TASK_CLOCK       = 1,
  PERF_COUNT_SW_PAGE_FAULTS      = 2,
  PERF_COUNT_SW_CONTEXT_SWITCHES = 3,
  PERF_COUNT_SW_CPU_MIGRATIONS   = 4,
  PERF_COUNT_SW_PAGE_FAULTS_MIN  = 5,
  PERF_COUNT_SW_PAGE_FAULTS_MAJ  = 6,
  PERF_COUNT_SW_ALIGNMENT_FAULTS = 7,
  PERF_COUNT_SW_EMULATION_FAULTS = 8,
  PERF_COUNT_SW_DUMMY            = 9,
  PERF_COUNT_SW_BPF_OUTPUT       = 10,
  PERF_COUNT_SW_MAX,
};

/* attr.config for PERF_TYPE_HW_CACHE
 * attr.config = id | (op_id << 8) | (op_result << 16)
 */

enum perf_hw_cache_id_e
{
  PERF_COUNT_HW_CACHE_L1D  = 0,
  PERF_COUNT_HW_CACHE_L1I  = 1,
  PERF_COUNT_HW_CACHE_LL   = 2,
  PERF_COUNT_HW_CACHE_DTLB = 3,
  PERF_COUNT_HW_CACHE_ITLB = 4,
  PERF_COUNT_HW_CACHE_BPU  = 5,
  PERF_COUNT_HW_CACHE_NODE = 6,
  PERF_COUNT_HW_CACHE_MAX,
};

enum perf_hw_cache_op_id_e
{
  PERF_COUNT_HW_CACHE_OP_READ     = 0,
  PERF_COUNT_HW_CACHE_OP_WRITE    = 1,
  PERF_COUNT_HW_CACHE_OP_PREFETCH = 2,
  PERF_COUNT_HW_CACHE_OP_MAX,
};

enum perf_hw_cache_op_result_id_e
{
  PERF_COUNT_HW_CACHE_RESULT_ACCESS = 0,
  PERF_COUNT_HW_CACHE_RESULT_MISS   = 1,
  PERF_COUNT_HW_CACHE_RESULT_MAX,
};

/* attr.read_format */

enum perf_event_read_format_e
{
  PERF_FORMAT_TOTAL_TIME_ENABLED = 1U << 0,
  PERF_FORMAT_TOTAL_TIME_RUNNING = 1U << 1,
  PERF_FORMAT_ID                 = 1U << 2,
  PERF_FORMAT_GROUP              = 1U << 3,
  PERF_FORMAT_LOST               = 1U << 4,
  PERF_FORMAT_MAX                = 1U << 5,
};

enum perf_event_state_e
{
  PERF_EVENT_STATE_OFF     = -1,
  PERF_EVENT_STATE_INACTIVE = 0,
  PERF_EVENT_STATE_ACTIVE   = 1,
};

struct hw_perf_event_s
{
  /* Hardware */

  struct
    {
      unsigned long config_base; /* Deposit event number and type */
      int idx;                   /* Event index */
      int flags;                 /* Event flags */
    };
  int state;                     /* Event status */
  uint64_t prev_count;           /* Value of the previous count */
};

struct perf_event_attr_s
{
  uint32_t type;
  uint32_t size;
  uint64_t config;
  union
    {
      uint64_t sample_period;
      uint64_t sample_freq;
    };
  uint64_t sample_type;
  uint64_t read_format;
  uint64_t disabled:1,
           inherit:1,
           pinned:1,
           exclusive:1,
           exclude_user:1,
           exclude_kernel:1,
           exclude_hv:1,
           exclude_idle:1,
           mmap:1,
           comm:1,
           freq:1,
           inherit_stat:1,
           enable_on_exec:1,
           task:1,
           watermark:1,
           context_switch:1,
           reserved_1:48;
  union
    {
      uint32_t wakeup_events;
      uint32_t wakeup_watermark;
    };
  union
    {
      uint64_t config1;
    };
};

struct perf_event_s
{
  int cpu;                                  /* CPU number this event belongs to */
  int oncpu;                                /* CPU number this event running on */
  int state;                                /* Event state */
  int attach_state;                         /* Attach states for this event */
  uint64_t count;                           /* Count vaule for this event */
  uint64_t child_count;                     /* Detached child events count */
  uint32_t sibling_num;                     /* sibling number for sibling_list */
  uint32_t child_num;                       /* Child number for child_list */
  struct perf_event_attr_s attr;            /* Perf event attr */
  FAR struct pmu_s *pmu;                    /* PMU entry this event used */
  FAR struct perf_event_context_s *ctx;     /* Event context this event belongs to */
  FAR struct pmu_event_context_s *pmuctx;   /* PMU event context */
  struct hw_perf_event_s hw;                /* Performance event hardware details */
  struct list_node event_node;              /* Connect to event_list in perf_event_context_s */
  struct list_node group_node;              /* Connect to group_list in perf_event_context_s */
  struct list_node sibling_list;            /* Used for list event in one group */
  struct list_node child_list;              /* Used for list inherit group leader event */
  mutex_t child_mutex;                      /* Mutex for child_list */
  FAR struct perf_event_s *group_leader;    /* Indicate the group leader of this event */
  FAR struct perf_event_s *parent_event;    /* Indicate the parent event of this event */
  uint64_t id;                              /* Event id */
  uint64_t total_time_enabled;
  uint64_t total_time_running;
};

struct perf_event_context_s
{
  struct list_node event_list;   /* Used for list perf event */
  struct list_node group_list;   /* Used for list group leader event */
  struct list_node pmu_ctx_list; /* Used for list PMU context */
  spinlock_t lock;               /* Lock for pref event context */
  FAR struct tcb_s *tcb;         /* The tcb context belongs to */
  uint32_t active_num;           /* Active event in this context */
};

struct pmu_event_context_s
{
  FAR struct pmu_s *pmu;
  FAR struct perf_event_context_s *ctx;
  struct list_node pmu_ctx_node;
  int refcount;
};

struct pmu_cpu_context_s
{
  struct pmu_event_context_s pmuctx;
  FAR struct pmu_event_context_s *task_pmuctx;
};

struct pmu_ops_s
{
  CODE void (*pmu_enable)(FAR struct pmu_s *pmu);
  CODE void (*pmu_disable)(FAR struct pmu_s *pmu);
  CODE int (*event_init)(FAR struct perf_event_s *event);
  CODE int (*event_add)(FAR struct perf_event_s *event, int flags);
  CODE void (*event_del)(FAR struct perf_event_s *event, int flags);
  CODE int (*event_start)(FAR struct perf_event_s *event, int flags);
  CODE int (*event_stop)(FAR struct perf_event_s *event, int flags);
  CODE int (*event_read)(FAR struct perf_event_s *event);
  CODE int (*event_idx)(FAR struct perf_event_s *event);
  CODE bool (*event_filter)(FAR struct pmu_s *pmu, int cpu);
};

struct pmu_s
{
  struct list_node node;
  FAR const char *name;
  int type;
  FAR struct pmu_ops_s *ops;
  struct pmu_cpu_context_s cpu_pmu_ctx[CONFIG_SMP_NCPUS];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: perf_event_init
 *
 * Description:
 *   Perf event initialization
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Init result
 *
 ****************************************************************************/

int perf_event_init(void);

/****************************************************************************
 * Name: perf_event_open
 *
 * Description:
 *   Create and open a perf event
 *
 * Input Parameters:
 *   attr     - Perf event attribute
 *   pid      - Task pid perf event bind
 *   cpu      - Cpu id perf event bind
 *   group_fd - Perf event fd for group leader
 *   flags    - Perf event flags
 *
 * Returned Value:
 *   Perf event fd
 *
 ****************************************************************************/

int perf_event_open(FAR struct perf_event_attr_s *attr, pid_t pid,
                    int cpu, int group_fd, unsigned long flags);

/****************************************************************************
 * Name: perf_event_overflow
 *
 * Description:
 *   Perf event overflow handler
 *
 * Input Parameters:
 *   event - Perf event
 *
 * Returned Value:
 *   Result
 *
 ****************************************************************************/

int perf_event_overflow(FAR struct perf_event_s *event);

/****************************************************************************
 * Name: perf_pmu_register
 *
 * Description:
 *   Register perf pmu
 *
 * Input Parameters:
 *   pmu  - Pmu entry.
 *   name - Pmu name.
 *   type - Perf type id.
 *
 * Returned Value:
 *   Register result
 *
 ****************************************************************************/

int perf_pmu_register(FAR struct pmu_s *pmu, FAR const char *name, int type);

/****************************************************************************
 * Name: perf_pmu_unregister
 *
 * Description:
 *   Unregister perf pmu
 *
 * Input Parameters:
 *   pmu  - Pmu entry.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_pmu_unregister(FAR struct pmu_s *pmu);

/****************************************************************************
 * Name: perf_event_task_init
 *
 * Description:
 *   Perf event task init, call during task initialization.
 *
 * Input Parameters:
 *   tcb   - Task for init.
 *
 * Returned Value:
 *   Initialization result
 *
 ****************************************************************************/

int perf_event_task_init(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: perf_event_task_exit
 *
 * Description:
 *   Perf event task free
 *
 * Input Parameters:
 *   tcb   - Task for free.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_exit(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: perf_event_task_sched_in
 *
 * Description:
 *   Record task sched in event
 *
 * Input Parameters:
 *   tcb - Refers to the head task which will be executed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_sched_in(FAR struct tcb_s *tcb);

/****************************************************************************
 * Name: perf_event_task_sched_out
 *
 * Description:
 *   Record task sched out event
 *
 * Input Parameters:
 *   tcb - Refers to the running task which will be blocked.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void perf_event_task_sched_out(FAR struct tcb_s *tcb);

#endif /* __INCLUDE_NUTTX_PERF_H */
