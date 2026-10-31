/****************************************************************************
 * include/nuttx/tls.h
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

#ifndef __INCLUDE_NUTTX_TLS_H
#define __INCLUDE_NUTTX_TLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <arch/arch.h>
#include <arch/types.h>

#include <nuttx/compiler.h>
#include <nuttx/cache.h>
#include <nuttx/atexit.h>
#include <nuttx/fs/fs.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>

#include <sys/types.h>

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

#ifdef CONFIG_FILE_STREAM
struct streamlist
{
  mutex_t                 sl_lock;   /* For thread safety */
  struct file_struct      sl_std[3];
  sq_queue_t              sl_queue;
};
#endif /* CONFIG_FILE_STREAM */

#ifdef __cplusplus
extern "C"
{
#endif

/* type tls_dtor_t **********************************************************/

/* Smallest addressable type that can hold the entire configured number of
 * TLS data indexes.
 */

typedef CODE void (*tls_dtor_t)(FAR void *);

#ifdef CONFIG_TLS_EMUTLS
/* The integer types of the emulated TLS of libgcc: a machine word, and an
 * integer as wide as a pointer.
 */

typedef unsigned int emutls_word_t __attribute__((mode(word)));
typedef unsigned int emutls_pointer_t __attribute__((mode(pointer)));
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

#ifdef CONFIG_FILE_STREAM
/* The state of getusershell(), allocated by libc at the first call */

struct usershell_s
{
  FAR FILE *us_file;         /* _PATH_SHELLS, or NULL: the default list */
  FAR char *us_line;         /* The line buffer of getline() */
  size_t    us_size;         /* The size of us_line */
  size_t    us_next;         /* The next shell of the default list */
};
#endif

#ifdef CONFIG_PTHREAD_ATFORK
/* This structure defines the pthread_atfork_s, which is used to manage
 * the funcs that operates on pthread_atfork() method
 */

struct pthread_atfork_s
{
  CODE void (*prepare)(void);
  CODE void (*child)(void);
  CODE void (*parent)(void);

  struct list_node node;
};
#endif

struct __res_state;

struct task_info_s
{
  rmutex_t        ta_lock;
#if CONFIG_TLS_TASK_NELEM > 0
  uintptr_t       ta_telem[CONFIG_TLS_TASK_NELEM]; /* Task local storage elements */
#endif
#if defined(CONFIG_TLS_NELEM) && CONFIG_TLS_NELEM > 0
  tls_dtor_t      ta_tlsdtor[CONFIG_TLS_NELEM]; /* List of TLS destructors      */
#endif
#ifndef CONFIG_BUILD_KERNEL
  struct getopt_s ta_getopt; /* Globals used by getopt() */
#  ifdef CONFIG_LIBC_LOCALE
  char            ta_domain[NAME_MAX]; /* Current domain for gettext */
#  endif
#endif
#ifdef CONFIG_NETDB_DNSCLIENT
  FAR struct __res_state *ta_res; /* Resolver state (_res), allocated by
                                   * libc at the first use
                                   */
#endif
#if CONFIG_LIBC_MAX_EXITFUNS > 0
  struct atexit_list_s ta_exit; /* Exit functions */
#endif
#ifdef CONFIG_FILE_STREAM
  struct streamlist ta_streamlist; /* Holds C buffered I/O info */

  /* The state of getusershell(), allocated by libc */

  FAR struct usershell_s *ta_usershell;
#endif

#ifdef CONFIG_PTHREAD_ATFORK
  struct list_node ta_atfork; /* Holds the pthread_atfork_s list */
#endif
  pid_t ta_pid; /* Process ID */
#ifndef CONFIG_DISABLE_ENVIRON
  /* Environment variables **************************************************/

  FAR char            **ta_envp;    /* The array that environ names         */
  FAR char            **ta_envarr;  /* The array that libc allocated        */
  FAR void             *ta_envmem;  /* Blocks that libc allocated, freed at
                                     * exit
                                     */
  size_t                ta_envpc;   /* Maximum entries of environment array */
  size_t                ta_envc;    /* Number of environment strings        */
#endif
#if defined(CONFIG_LIBC_ELF) || defined(CONFIG_BINFMT_LOADABLE)
  rmutex_t         ta_modlock; /* Protects ta_modules */
  struct list_node ta_modules; /* Objects this task group loaded */
#endif
};

/* struct tls_cleanup_s *****************************************************/

/* This structure describes one element of the pthread cleanup stack */

typedef CODE void (*tls_cleanup_t)(FAR void *arg);

struct tls_cleanup_s
{
  tls_cleanup_t tc_cleaner; /* Cleanup callback address */
  FAR void *tc_arg;         /* Argument that accompanies the callback */
};

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
  FAR struct task_info_s *tl_task;

#if defined(CONFIG_TLS_NELEM) && CONFIG_TLS_NELEM > 0
  uintptr_t tl_elem[CONFIG_TLS_NELEM]; /* TLS elements */
#endif

  /* tl_tos   - The index to the next available entry at the top of the
   *            stack.
   * tl_stack - The pre-allocated clean-up stack memory.
   */

#if CONFIG_TLS_NCLEANUP > 0
  uint8_t tl_tos;
  struct tls_cleanup_s tl_stack[CONFIG_TLS_NCLEANUP];
#endif

  uint8_t tl_cpstate;                  /* Cancellation state */

#ifdef CONFIG_CANCELLATION_POINTS
  int16_t tl_cpcount;                  /* Nested cancellation point count */
#endif

  uint16_t tl_size;                    /* Actual size with alignments */
  int tl_errno;                        /* Per-thread error number */
  pid_t tl_tid;                        /* Thread ID */
  FAR char **tl_argv;                  /* Arguments first string */
#ifdef CONFIG_TLS_EMUTLS
  FAR void *tl_emutls;                 /* Copies of __thread variables */
#endif

  /* Robust mutex support ***************************************************/

#if !defined(CONFIG_DISABLE_PTHREAD) && !defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
  FAR struct pthread_mutex_s *tl_mhead;   /* List of mutexes held by thread  */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if CONFIG_TLS_TASK_NELEM > 0

/****************************************************************************
 * Name: task_tls_alloc
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

#if defined(CONFIG_TLS_NELEM) && CONFIG_TLS_NELEM > 0
void tls_destruct(void);
#endif

/****************************************************************************
 * Name: __wrap___emutls_get_address and __wrap___emutls_register_common
 *
 * Description:
 *   The per-thread emulated TLS of libc.  --wrap sends the calls that GCC
 *   makes to __emutls_get_address() and __emutls_register_common() here.
 *   The system symbol table (libc.csv) needs these prototypes.
 *
 ****************************************************************************/

#ifdef CONFIG_TLS_EMUTLS
FAR void *__wrap___emutls_get_address(FAR void *arg);
void __wrap___emutls_register_common(FAR void *arg, emutls_word_t size,
                                     emutls_word_t align, FAR void *templ);
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

#if defined(CONFIG_LIBC_ELF) || defined(CONFIG_BINFMT_LOADABLE)
/****************************************************************************
 * Name: task_modules_lock
 *
 * Description:
 *   Lock the list of the objects that the calling task group loaded
 *   (dlopen() and the libraries its program needs) and return it.  The
 *   lock is recursive: the constructors of a library run with it held, and
 *   they can call dlopen().
 *
 ****************************************************************************/

static inline FAR struct list_node *task_modules_lock(void)
{
  FAR struct task_info_s *info = task_get_info();
  int ret;

  /* A signal must not make the caller run without the lock */

  do
    {
      ret = nxrmutex_lock(&info->ta_modlock);
    }
  while (ret == -EINTR);

  return &info->ta_modules;
}

/****************************************************************************
 * Name: task_modules_unlock
 *
 * Description:
 *   Unlock the list of the objects that the calling task group loaded.
 *
 ****************************************************************************/

static inline void task_modules_unlock(void)
{
  nxrmutex_unlock(&task_get_info()->ta_modlock);
}
#endif

#if CONFIG_TLS_NCLEANUP > 0
/****************************************************************************
 * Name: tls_cleanup_push
 *
 * Description:
 *   Push a new entry onto the cleanup stack
 *
 * Input Parameters:
 *   tls     - The TLS data structure
 *   routine - The cleanup routine to be called
 *   arg     - An argument that will be passed to the cleanup routine
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tls_cleanup_push(FAR struct tls_info_s *tls,
                      tls_cleanup_t routine, FAR void *arg);

/****************************************************************************
 * Name: tls_cleanup_pop
 *
 * Description:
 *   Pop the top entry from the cleanup stack and execute the cleanup
 *
 * Input Parameters:
 *   tls     - The TLS data structure
 *   execute - Execute the cleanup routine (true) or just remove the
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tls_cleanup_pop(FAR struct tls_info_s *tls, int execute);

/****************************************************************************
 * Name: tls_cleanup_popall
 *
 * Description:
 *   Pop and execute all cleanup stack entries
 *
 * Input Parameters:
 *   tls     - The TLS data structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tls_cleanup_popall(FAR struct tls_info_s *tls);

#else
#  define tls_cleanup_push(tls, routine, arg) ((void)(tls), (void)(routine), (void)(arg))
#  define tls_cleanup_pop(tls, execute) ((void)(tls), (void)(execute))
#  define tls_cleanup_popall(tls) ((void)(tls))
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_TLS_H */
