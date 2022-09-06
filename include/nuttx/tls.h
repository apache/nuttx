/****************************************************************************
 * include/nuttx/tls.h
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

#ifndef __INCLUDE_NUTTX_TLS_H
#define __INCLUDE_NUTTX_TLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/atexit.h>

#include <sys/types.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_TLS_ALIGNED
#  ifndef CONFIG_TLS_LOG2_MAXSTACK
#    error CONFIG_TLS_LOG2_MAXSTACK is not defined
#  endif
#endif

#ifndef CONFIG_TLS_NELEM
#  warning CONFIG_TLS_NELEM is not defined
#  define CONFIG_TLS_NELEM 0
#endif

/* TLS Definitions **********************************************************/

#ifdef CONFIG_TLS_ALIGNED
#  define TLS_STACK_ALIGN  (1L << CONFIG_TLS_LOG2_MAXSTACK)
#  define TLS_STACK_MASK   (TLS_STACK_ALIGN - 1)
#  define TLS_MAXSTACK     (TLS_STACK_ALIGN)
#  define TLS_INFO(sp)     ((FAR struct tls_info_s *)((sp) & ~TLS_STACK_MASK))
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* type tls_ndxset_t & tls_dtor_t *******************************************/

/* Smallest addressable type that can hold the entire configured number of
 * TLS data indexes.
 */

#if CONFIG_TLS_NELEM > 0
#  if CONFIG_TLS_NELEM > 64
#    error Too many TLS elements
#  elif CONFIG_TLS_NELEM > 32
     typedef uint64_t tls_ndxset_t;
#  elif CONFIG_TLS_NELEM > 16
     typedef uint32_t tls_ndxset_t;
#  elif CONFIG_TLS_NELEM > 8
     typedef uint16_t tls_ndxset_t;
#  else
     typedef uint8_t tls_ndxset_t;
#  endif

typedef CODE void (*tls_dtor_t)(FAR void *);

#endif

#if CONFIG_TLS_TASK_NELEM > 0
#  if CONFIG_TLS_TASK_NELEM > 64
#    error Too many TLS elements
#  elif CONFIG_TLS_TASK_NELEM > 32
     typedef uint64_t tls_task_ndxset_t;
#  elif CONFIG_TLS_TASK_NELEM > 16
     typedef uint32_t tls_task_ndxset_t;
#  elif CONFIG_TLS_TASK_NELEM > 8
     typedef uint16_t tls_task_ndxset_t;
#  else
     typedef uint8_t tls_task_ndxset_t;
#  endif
#endif

/* This structure encapsulates all variables associated with getopt(). */

struct getopt_s
{
  /* Part of the implementation of the public getopt() interface */

  FAR char *go_optarg;       /* Optional argument following option */
  int       go_opterr;       /* Print error message */
  int       go_optind;       /* Index into argv */
  int       go_optopt;       /* unrecognized option character */

  /* Internal getopt() state */

  FAR char *go_optptr;       /* Current parsing location */
  bool      go_binitialized; /* true:  getopt() has been initialized */
};

struct task_info_s
{
  mutex_t         ta_lock;
  FAR char      **argv;                         /* Name+start-up parameters     */
#if CONFIG_TLS_TASK_NELEM > 0
  uintptr_t       ta_telem[CONFIG_TLS_TASK_NELEM]; /* Task local storage elements */
#endif
#if CONFIG_TLS_NELEM > 0
  tls_ndxset_t    ta_tlsset;                    /* Set of TLS indexes allocated */
  tls_dtor_t      ta_tlsdtor[CONFIG_TLS_NELEM]; /* List of TLS destructors      */
#endif
#ifndef CONFIG_BUILD_KERNEL
  struct getopt_s ta_getopt; /* Globals used by getopt() */
  mode_t          ta_umask;  /* File mode creation mask */
#  ifdef CONFIG_LIBC_LOCALE
  char            ta_domain[NAME_MAX]; /* Current domain for gettext */
#  endif
#endif
#if CONFIG_LIBC_MAX_EXITFUNS > 0
  struct atexit_list_s ta_exit; /* Exit functions */
#endif
};

/* struct pthread_cleanup_s *************************************************/

/* This structure describes one element of the pthread cleanup stack */

#ifdef CONFIG_PTHREAD_CLEANUP
struct pthread_cleanup_s
{
  pthread_cleanup_t pc_cleaner;     /* Cleanup callback address */
  FAR void *pc_arg;                 /* Argument that accompanies the callback */
};
#endif

/* When TLS is enabled, up_createstack() will align allocated stacks to the
 * TLS_STACK_ALIGN value.  An instance of the following structure will be
 * implicitly positioned at the "lower" end of the stack.  Assuming a
 * "push down" stack, this is at the "far" end of the stack (and can be
 * clobbered if the stack overflows).
 *
 * If an MCU has a "push up" then that TLS structure will lie at the top
 * of the stack and stack allocation and initialization logic must take
 * care to preserve this structure content.
 *
 * The stack memory is fully accessible to user mode threads.  TLS is not
 * available from interrupt handlers (nor from the IDLE thread).
 *
 * The following diagram represent the typical stack layout:
 *
 *      Push Down             Push Up
 *   +-------------+      +-------------+ <- Stack memory allocation
 *   | Task Data*  |      | Task Data*  |
 *   +-------------+      +-------------+
 *   |  TLS Data   |      |  TLS Data   |
 *   +-------------+      +-------------+
 *   |  Arguments  |      |  Arguments  |
 *   +-------------+      +-------------+ |
 *   |             |      |             | v
 *   | Available   |      | Available   |
 *   |   Stack     |      |   Stack     |
 *   |             |      |             |
 *   |             |      |             |
 *   |             | ^    |             |
 *   +-------------+ |    +-------------+
 *
 *  Task data is a pointer that pointed to a user space memory region.
 */

struct tls_info_s
{
  FAR struct task_info_s * tl_task;

#if CONFIG_TLS_NELEM > 0
  uintptr_t tl_elem[CONFIG_TLS_NELEM]; /* TLS elements */
#endif

#ifdef CONFIG_PTHREAD_CLEANUP
  /* tos   - The index to the next available entry at the top of the stack.
   * stack - The pre-allocated clean-up stack memory.
   */

  uint8_t tos;
  struct pthread_cleanup_s stack[CONFIG_PTHREAD_CLEANUP_STACKSIZE];
#endif

  int tl_errno;                        /* Per-thread error number */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if CONFIG_TLS_TASK_NELEM > 0

/****************************************************************************
 * Name: task_tls_allocs
 *
 * Description:
 *   Allocate a global-unique task local storage data index
 *
 * Input Parameters:
 *   dtor     - The destructor of task local storage data element
 *
 * Returned Value:
 *   A TLS index that is unique.
 *
 ****************************************************************************/

int task_tls_alloc(tls_dtor_t dtor);

/****************************************************************************
 * Name: task_tls_destruct
 *
 * Description:
 *   Destruct all TLS data element associated with allocated key
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

void task_tls_destruct(void);

/****************************************************************************
 * Name: task_tls_set_value
 *
 * Description:
 *   Set the task local storage element associated with the 'tlsindex' to
 *   'tlsvalue'
 *
 * Input Parameters:
 *   tlsindex - Index of task local storage data element to set
 *   tlsvalue - The new value of the task local storage data element
 *
 * Returned Value:
 *   Zero is returned on success, a negated errno value is return on
 *   failure:
 *
 *     EINVAL - tlsindex is not in range.
 *
 ****************************************************************************/

int task_tls_set_value(int tlsindex, uintptr_t tlsvalue);

/****************************************************************************
 * Name: task_tls_get_value
 *
 * Description:
 *   Return an the task local storage data value associated with 'tlsindx'
 *
 * Input Parameters:
 *   tlsindex - Index of task local storage data element to return
 *
 * Returned Value:
 *   The value of TLS element associated with 'tlsindex'. Errors are not
 *   reported.  Zero is returned in the event of an error, but zero may also
 *   be valid value and returned when there is no error.  The only possible
 *   error would be if tlsindex < 0 or tlsindex >=CONFIG_TLS_TASK_NELEM.
 *
 ****************************************************************************/

uintptr_t task_tls_get_value(int tlsindex);
#endif

/****************************************************************************
 * Name: tls_get_info
 *
 * Description:
 *   Return a reference to the tls_info_s structure.  This is used as part
 *   of the internal implementation of tls_get/set_elem() and ONLY for the
 *   where CONFIG_TLS_ALIGNED is *not* defined
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the thread-specific tls_info_s structure is return on
 *   success.  NULL would be returned in the event of any failure.
 *
 ****************************************************************************/

#if defined(up_tls_info)
#  define tls_get_info() up_tls_info()
#elif defined(CONFIG_TLS_ALIGNED) && !defined(__KERNEL__)
#  define tls_get_info() TLS_INFO(up_getsp())
#else
FAR struct tls_info_s *tls_get_info(void);
#endif

/****************************************************************************
 * Name: tls_destruct
 *
 * Description:
 *   Destruct all TLS data element associated with allocated key
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if CONFIG_TLS_NELEM > 0
void tls_destruct(void);
#endif

/****************************************************************************
 * Name: task_get_info
 *
 * Description:
 *   Return a reference to the task_info_s structure.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A reference to the task-specific task_info_s structure is return on
 *   success.  NULL would be returned in the event of any failure.
 *
 ****************************************************************************/

FAR struct task_info_s *task_get_info(void);

#endif /* __INCLUDE_NUTTX_TLS_H */
