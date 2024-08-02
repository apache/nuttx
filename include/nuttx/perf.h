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
#include <nuttx/circbuf.h>
#include <nuttx/wdog.h>

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
  PERF_EVENT_STATE_EXIT     = -3,
  PERF_EVENT_STATE_ERROR    = -2,
  PERF_EVENT_STATE_OFF      = -1,
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
  struct wdog_s waitdog;
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

struct perf_buffer_s
{
  struct circbuf_s rb;
  uint32_t ref_count;
  spinlock_t lock;
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
  struct list_node sw_list;                 /* Used for list event in swevent_manager_s */
  mutex_t child_mutex;                      /* Mutex for child_list */
  FAR struct perf_event_s *group_leader;    /* Indicate the group leader of this event */
  FAR struct perf_event_s *parent_event;    /* Indicate the parent event of this event */
  uint64_t id;                              /* Event id */
  uint64_t total_time_enabled;
  uint64_t total_time_running;
  FAR struct perf_buffer_s *buf;
  FAR struct pollfd *pfd;
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
  CODE int (*event_match)(FAR struct perf_event_s *event);
};

struct pmu_s
{
  struct list_node node;
  FAR const char *name;
  int type;
  FAR struct pmu_ops_s *ops;
  struct pmu_cpu_context_s cpu_pmu_ctx[CONFIG_SMP_NCPUS];
};

enum perf_event_sample_format_e
{
  PERF_SAMPLE_IP             = 1U << 0,
  PERF_SAMPLE_TID            = 1U << 1,
  PERF_SAMPLE_TIME           = 1U << 2,
  PERF_SAMPLE_ADDR           = 1U << 3,
  PERF_SAMPLE_READ           = 1U << 4,
  PERF_SAMPLE_CALLCHAIN      = 1U << 5,
  PERF_SAMPLE_ID             = 1U << 6,
  PERF_SAMPLE_CPU            = 1U << 7,
  PERF_SAMPLE_PERIOD         = 1U << 8,
  PERF_SAMPLE_STREAM_ID      = 1U << 9,
  PERF_SAMPLE_RAW            = 1U << 10,
  PERF_SAMPLE_BRANCH_STACK   = 1U << 11,
  PERF_SAMPLE_REGS_USER      = 1U << 12,
  PERF_SAMPLE_STACK_USER     = 1U << 13,
  PERF_SAMPLE_WEIGHT         = 1U << 14,
  PERF_SAMPLE_DATA_SRC       = 1U << 15,
  PERF_SAMPLE_IDENTIFIER     = 1U << 16,
  PERF_SAMPLE_TRANSACTION    = 1U << 17,
  PERF_SAMPLE_REGS_INTR      = 1U << 18,
  PERF_SAMPLE_PHYS_ADDR      = 1U << 19,
  PERF_SAMPLE_AUX            = 1U << 20,
  PERF_SAMPLE_CGROUP         = 1U << 21,
  PERF_SAMPLE_DATA_PAGE_SIZE = 1U << 22,
  PERF_SAMPLE_CODE_PAGE_SIZE = 1U << 23,
  PERF_SAMPLE_WEIGHT_STRUCT  = 1U << 24,

  PERF_SAMPLE_MAX            = 1U << 25,    /* non-ABI */
};

enum perf_event_type_e
{
/* If perf_event_attr.sample_id_all is set then all event types will
 * have the sample_type selected fields related to where/when
 * (identity) an event took place (TID, TIME, ID, STREAM_ID, CPU,
 * IDENTIFIER) described in PERF_RECORD_SAMPLE below, it will be stashed
 * just after the perf_event_header_s and the fields already present for
 * the existing fields, i.e. at the end of the payload. That way a newer
 * perf.data file will be supported by older perf tools, with these new
 * optional fields being ignored.
 *
 * struct sample_id {
 *   { u32      pid, tid; } && PERF_SAMPLE_TID
 *   { u64      time;     } && PERF_SAMPLE_TIME
 *   { u64      id;       } && PERF_SAMPLE_ID
 *   { u64      stream_id;} && PERF_SAMPLE_STREAM_ID
 *   { u32      cpu, res; } && PERF_SAMPLE_CPU
 *  { u64      id;    } && PERF_SAMPLE_IDENTIFIER
 * } && perf_event_attr::sample_id_all
 *
 * Note that PERF_SAMPLE_IDENTIFIER duplicates PERF_SAMPLE_ID.  The
 * advantage of PERF_SAMPLE_IDENTIFIER is that its position is fixed
 * relative to header.size.
 */

/* The MMAP events record the PROT_EXEC mappings so that we can
 * correlate userspace IPs to code. They have the following structure:
 *
 * struct {
 *  struct perf_event_header_s  header;
 *
 *  u32        pid, tid;
 *  u64        addr;
 *  u64        len;
 *  u64        pgoff;
 *  char        filename[];
 *   struct sample_id    sample_id;
 * };
 */

  PERF_RECORD_MMAP      = 1,

/* struct {
 *  struct perf_event_header_s  header;
 *  u64        id;
 *  u64        lost;
 *   struct sample_id    sample_id;
 * };
 */

  PERF_RECORD_LOST      = 2,

/* struct {
 *   struct perf_event_header_s  header;
 *   u32        pid, tid;
 *   char        comm[];
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_COMM      = 3,

/* struct {
 *   struct perf_event_header_s  header;
 *   u32        pid, ppid;
 *   u32        tid, ptid;
 *   u64        time;
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_EXIT      = 4,

/* struct {
 *   struct perf_event_header_s  header;
 *   u64        time;
 *   u64        id;
 *   u64        stream_id;
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_THROTTLE      = 5,
  PERF_RECORD_UNTHROTTLE      = 6,

/* struct {
 *   struct perf_event_header_s  header;
 *   u32        pid, ppid;
 *   u32        tid, ptid;
 *   u64        time;
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_FORK      = 7,

/* struct {
 *   struct perf_event_header_s  header;
 *   u32        pid, tid;
 *   struct read_format    values;
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_READ      = 8,

/* struct {
 *   struct perf_event_header_s  header;
 *   #
 *   # Note that PERF_SAMPLE_IDENTIFIER duplicates PERF_SAMPLE_ID.
 *   # The advantage of PERF_SAMPLE_IDENTIFIER is that its position
 *   # is fixed relative to header.
 *   #
 *   { u64      id;    } && PERF_SAMPLE_IDENTIFIER
 *   { u64      ip;    } && PERF_SAMPLE_IP
 *   { u32      pid, tid; } && PERF_SAMPLE_TID
 *   { u64      time;     } && PERF_SAMPLE_TIME
 *   { u64      addr;     } && PERF_SAMPLE_ADDR
 *   { u64      id;    } && PERF_SAMPLE_ID
 *   { u64      stream_id;} && PERF_SAMPLE_STREAM_ID
 *   { u32      cpu, res; } && PERF_SAMPLE_CPU
 *   { u64      period;   } && PERF_SAMPLE_PERIOD
 *
 *   { struct read_format  values;    } && PERF_SAMPLE_READ
 *
 *   { u64      nr,
 *     u64      ips[nr];  } && PERF_SAMPLE_CALLCHAIN
 *
 *   #
 *   # The RAW record below is opaque data wrt the ABI
 *   #
 *   # That is, the ABI doesn't make any promises wrt to
 *   # the stability of its content, it may vary depending
 *   # on event, hardware, kernel version and phase of
 *   # the moon.
 *   #
 *   # In other words, PERF_SAMPLE_RAW contents are not an ABI.
 *   #
 *
 *   { u32      size;
 *     char                  data[size];}&& PERF_SAMPLE_RAW
 *
 *   { u64                   nr;
 *     { u64  hw_idx; } && PERF_SAMPLE_BRANCH_HW_INDEX
 *         { u64 from, to, flags } lbr[nr];
 *       } && PERF_SAMPLE_BRANCH_STACK
 *
 *    { u64      abi; # enum perf_sample_regs_abi
 *      u64      regs[weight(mask)]; } && PERF_SAMPLE_REGS_USER
 *
 *    { u64      size;
 *      char      data[size];
 *      u64      dyn_size; } && PERF_SAMPLE_STACK_USER
 *
 *   { union perf_sample_weight
 *    {
 *     u64    full; && PERF_SAMPLE_WEIGHT
 *   #if defined(__LITTLE_ENDIAN_BITFIELD)
 *     struct {
 *       u32  var1_dw;
 *       u16  var2_w;
 *       u16  var3_w;
 *     } && PERF_SAMPLE_WEIGHT_STRUCT
 *   #elif defined(__BIG_ENDIAN_BITFIELD)
 *     struct {
 *       u16  var3_w;
 *       u16  var2_w;
 *       u32  var1_dw;
 *     } && PERF_SAMPLE_WEIGHT_STRUCT
 *   #endif
 *    }
 *   }
 *   { u64      data_src; } && PERF_SAMPLE_DATA_SRC
 *   { u64      transaction; } && PERF_SAMPLE_TRANSACTION
 *   { u64      abi; # enum perf_sample_regs_abi
 *     u64      regs[weight(mask)]; } && PERF_SAMPLE_REGS_INTR
 *   { u64      phys_addr;} && PERF_SAMPLE_PHYS_ADDR
 *   { u64      size;
 *     char      data[size]; } && PERF_SAMPLE_AUX
 *   { u64      data_page_size;} && PERF_SAMPLE_DATA_PAGE_SIZE
 *   { u64      code_page_size;} && PERF_SAMPLE_CODE_PAGE_SIZE
 *  };
 */

  PERF_RECORD_SAMPLE      = 9,

/* The MMAP2 records are an augmented version of MMAP, they add
 *  maj, min, ino numbers to be used to uniquely identify each mapping
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *
 *   u32        pid, tid;
 *   u64        addr;
 *   u64        len;
 *   u64        pgoff;
 *   union {
 *     struct {
 *       u32    maj;
 *       u32    min;
 *       u64    ino;
 *       u64    ino_generation;
 *     };
 *     struct {
 *       u8    build_id_size;
 *       u8    __reserved_1;
 *       u16    __reserved_2;
 *       u8    build_id[20];
 *     };
 *   };
 *   u32        prot, flags;
 *   char        filename[];
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_MMAP2      = 10,

/* Records that new data landed in the AUX buffer part.
 *
 *  struct {
 *    struct perf_event_header_s  header;
 *
 *    u64        aux_offset;
 *    u64        aux_size;
 *   u64        flags;
 *    struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_AUX        = 11,

/* Indicates that instruction trace has started
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   u32        pid;
 *   u32        tid;
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_ITRACE_START    = 12,

/* Records the dropped/lost sample number.
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *
 *   u64        lost;
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_LOST_SAMPLES    = 13,

/* Records a context switch in or out (flagged by
 *  PERF_RECORD_MISC_SWITCH_OUT). See also
 *  PERF_RECORD_SWITCH_CPU_WIDE.
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_SWITCH      = 14,

/* CPU-wide version of PERF_RECORD_SWITCH with next_prev_pid and
 *  next_prev_tid that are the next (switching out) or previous
 *  (switching in) pid/tid.
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   u32        next_prev_pid;
 *   u32        next_prev_tid;
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_SWITCH_CPU_WIDE    = 15,

/* struct {
 *   struct perf_event_header_s  header;
 *   u32        pid;
 *   u32        tid;
 *   u64        nr_namespaces;
 *   { u64        dev, inode; } [nr_namespaces];
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_NAMESPACES      = 16,

/* Record ksymbol register/unregister events:
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   u64        addr;
 *   u32        len;
 *   u16        ksym_type;
 *   u16        flags;
 *   char        name[];
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_KSYMBOL      = 17,

/* Record bpf events:
 *   enum perf_bpf_event_type {
 *   PERF_BPF_EVENT_UNKNOWN    = 0,
 *   PERF_BPF_EVENT_PROG_LOAD  = 1,
 *   PERF_BPF_EVENT_PROG_UNLOAD  = 2,
 *   };
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   u16        type;
 *   u16        flags;
 *   u32        id;
 *   u8        tag[BPF_TAG_SIZE];
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_BPF_EVENT      = 18,

/* struct {
 *   struct perf_event_header_s  header;
 *   u64        id;
 *   char        path[];
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_CGROUP      = 19,

/* Records changes to kernel text i.e. self-modified code. 'old_len' is
 *  the number of old bytes, 'new_len' is the number of new bytes. Either
 *  'old_len' or 'new_len' may be zero to indicate, for example, the
 *  addition or removal of a trampoline. 'bytes' contains the old bytes
 *  followed immediately by the new bytes.
 *
 *  struct {
 *   struct perf_event_header_s  header;
 *   u64        addr;
 *   u16        old_len;
 *   u16        new_len;
 *   u8        bytes[];
 *   struct sample_id    sample_id;
 *  };
 */

  PERF_RECORD_TEXT_POKE      = 20,

  PERF_RECORD_MAX,      /* non-ABI */
};

struct perf_sample_data_s
{
/* Fields set by perf_sample_data_init() unconditionally,
 *  group so as to minimize the cachelines touched.
 */

  uint64_t sample_flags;
  uint64_t period;
  uint64_t dyn_size;

/* Fields commonly set by __perf_event_header__init_id(),
 *  group so as to minimize the cachelines touched.
 */

  uint64_t type;
  struct
  {
    pid_t pid;
    pid_t tid;
  } tid_entry;
  uint64_t time;
  uint64_t id;
  struct
  {
    uint32_t cpu;
    uint32_t reserved;
  } cpu_entry;

/* The other fields, optionally {set,used} by
 *  perf_{prepare,output}_sample().
 */

  uint64_t ip;
  struct perf_callchain_entry *callchain;
  struct perf_raw_record *raw;
  struct perf_branch_stack *br_stack;
  uint64_t *br_stack_cntr;
  uint64_t txn;

  uint64_t stack_user_size;

  uint64_t stream_id;
  uint64_t cgroup;
  uint64_t addr;
  uint64_t phys_addr;
  uint64_t data_page_size;
  uint64_t code_page_size;
  uint64_t aux_size;
};

struct perf_event_header_s
{
  uint32_t  type;
  uint16_t  misc;
  uint16_t  size;
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
