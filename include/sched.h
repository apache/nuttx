/********************************************************************************
 * include/sched.h
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
 ********************************************************************************/

#ifndef __INCLUDE_SCHED_H
#define __INCLUDE_SCHED_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include <time.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* Task Management Definitions **************************************************/

/* POSIX-like scheduling policies */

#define SCHED_FIFO                1  /* FIFO priority scheduling policy */
#define SCHED_RR                  2  /* Round robin scheduling policy */
#define SCHED_SPORADIC            3  /* Sporadic scheduling policy */
#define SCHED_OTHER               4  /* Not supported */

/* Maximum number of SCHED_SPORADIC replenishments */

#define SS_REPL_MAX               CONFIG_SCHED_SPORADIC_MAXREPL

/* Cancellation definitions *****************************************************/

/* Cancellation states used by task_setcancelstate() */

#define TASK_CANCEL_ENABLE        (0)
#define TASK_CANCEL_DISABLE       (1)

/* Cancellation types used by task_setcanceltype() */

#define TASK_CANCEL_DEFERRED      (0)
#define TASK_CANCEL_ASYNCHRONOUS  (1)

/* Pthread definitions **********************************************************/

#define PTHREAD_KEYS_MAX          CONFIG_TLS_NELEM

/* CPU affinity mask helpers ****************************************************/

/* These are not standard but are defined for Linux compatibility */

#ifdef CONFIG_SMP

/* void CPU_ZERO(FAR cpu_set_t *set); */

#  define CPU_ZERO(s) do { *(s) = 0; } while (0)

/* void CPU_SET(int cpu, FAR cpu_set_t *set); */

#  define CPU_SET(c,s) do { *(s) |= (1 << (c)); } while (0)

/* void CPU_CLR(int cpu, FAR cpu_set_t *set); */

#  define CPU_CLR(c,s) do { *(s) &= ~(1 << (c)); } while (0)

/* int  CPU_ISSET(int cpu, FAR const cpu_set_t *set); */

#  define CPU_ISSET(c,s) ((*(s) & (1 << (c))) != 0)

/* int CPU_COUNT(FAR const cpu_set_t *set); */

#  define CPU_COUNT(s) popcountl(*s)

/* void CPU_AND(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_AND(d,s1,s2) do { *(d) = *(s1) & *(s2); } while (0)

/* void CPU_OR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *             FAR const cpu_set_t *srcset2);
 */

#  define CPU_OR(d,s1,s2) do { *(d) = *(s1) | *(s2); } while (0)

/* void CPU_XOR(FAR cpu_set_t *destset, FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_XOR(d,s1,s2) do { *(d) = *(s1) ^ *(s2); } while (0)

/* int CPU_EQUAL(FAR const cpu_set_t *set1, FAR const cpu_set_t *set2); */

#  define CPU_EQUAL(s1,s2) (*(s2) == *(s2))

/* REVISIT: Variably sized CPU sets are not supported */

/* FAR cpu_set_t *CPU_ALLOC(int num_cpus); */

#  define CPU_ALLOC(n) (FAR cpu_set_t *)malloc(sizeof(cpu_set_t));

/* void CPU_FREE(cpu_set_t *set); */

#  define CPU_FREE(s) free(s)

/* size_t CPU_ALLOC_SIZE(int num_cpus); */

#  define CPU_ALLOC_SIZE(n) sizeof(cpu_set_t)

/* void CPU_ZERO_S(size_t setsize, FAR cpu_set_t *set); */

#  define CPU_ZERO_S(n,s) CPU_ZERO_S(s)

/* void CPU_SET_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#  define CPU_SET_S(c,n,s) CPU_SET(c,s)

/* void CPU_CLR_S(int cpu, size_t setsize, FAR cpu_set_t *set); */

#  define CPU_CLR_S(c,n,s) CPU_CLR(c,s)

/* int CPU_ISSET_S(int cpu, size_t setsize, FAR const cpu_set_t *set); */

#  define CPU_ISSET_S(c,n,s) CPU_ISSET(c,s)

/* int CPU_COUNT_S(size_t setsize, FAR const cpu_set_t *set); */

#  define CPU_COUNT_S(n,s) CPU_COUNT(s)

/* void CPU_AND_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#  define CPU_AND_S(n,d,s1,s2) CPU_AND(d,s1,s2)

/* void CPU_OR_S(size_t setsize, FAR cpu_set_t *destset,
 *              FAR const cpu_set_t *srcset1,
 *              FAR const cpu_set_t *srcset2);
 */

#  define CPU_OR_S(n,d,s1,s2) CPU_OR(d,s1,s2)

/* void CPU_XOR_S(size_t setsize, FAR cpu_set_t *destset,
 *                FAR const cpu_set_t *srcset1,
 *                FAR const cpu_set_t *srcset2);
 */

#  define CPU_XOR_S(n,d,s1,s2) CPU_XOR(d,s1,s2)

/* int CPU_EQUAL_S(size_t setsize, FAR const cpu_set_t *set1,
 *                 FAR const cpu_set_t *set2);
 */

#  define CPU_EQUAL_S(n,s1,s2) CPU_EQUAL(s1,s2)

#endif /* CONFIG_SMP */

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

/* This is the POSIX-like scheduling parameter structure */

struct sched_param
{
  int sched_priority;                   /* Base thread priority */

#ifdef CONFIG_SCHED_SPORADIC
  int sched_ss_low_priority;            /* Low scheduling priority for sporadic
                                         * server */
  struct timespec sched_ss_repl_period; /* Replenishment period for sporadic
                                         * server. */
  struct timespec sched_ss_init_budget; /* Initial budget for sporadic server */
  int sched_ss_max_repl;                /* Maximum pending replenishments for
                                         * sporadic server. */
#endif
};

/********************************************************************************
 * Public Data
 ********************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/* Task Control Interfaces (non-standard) */

#ifndef CONFIG_BUILD_KERNEL
int    task_create(FAR const char *name, int priority, int stack_size,
                   main_t entry, FAR char * const argv[]);
int    task_create_with_stack(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              main_t entry, FAR char * const argv[]);
#endif
int    task_delete(pid_t pid);
int    task_restart(pid_t pid);

int    task_setcancelstate(int state, FAR int *oldstate);
int    task_setcanceltype(int type, FAR int *oldtype);
void   task_testcancel(void);

/* Task Scheduling Interfaces (based on POSIX APIs) */

int    sched_setparam(pid_t pid, FAR const struct sched_param *param);
int    sched_getparam(pid_t pid, FAR struct sched_param *param);
int    sched_setscheduler(pid_t pid, int policy,
                          FAR const struct sched_param *param);
int    sched_getscheduler(pid_t pid);
int    sched_yield(void);
int    sched_get_priority_max(int policy);
int    sched_get_priority_min(int policy);
int    sched_rr_get_interval(pid_t pid, FAR struct timespec *interval);

#ifdef CONFIG_SMP
/* Task affinity */

int    sched_setaffinity(pid_t pid, size_t cpusetsize,
                         FAR const cpu_set_t *mask);
int    sched_getaffinity(pid_t pid, size_t cpusetsize, FAR cpu_set_t *mask);
int    sched_cpucount(FAR const cpu_set_t *set);
int    sched_getcpu(void);
#endif /* CONFIG_SMP */

/* Task Switching Interfaces (non-standard) */

int    sched_lock(void);
int    sched_unlock(void);
int    sched_lockcount(void);

/* Queries */

bool   sched_idletask(void);

/* Task Backtrace */

int    sched_backtrace(pid_t tid, FAR void **buffer, int size, int skip);
void   sched_dumpstack(pid_t tid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_SCHED_H */
