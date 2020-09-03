/****************************************************************************
 * arch/sim/src/sim/up_ucontext.c
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

/* This define is required to compile on OSX */

#ifndef _XOPEN_SOURCE
#  define _XOPEN_SOURCE           (500)
#endif

#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <ucontext.h>

/****************************************************************************
 * Preprocessor Definition
 ****************************************************************************/

/* The minimum ucontext stack size */

#define MIN_UCONTEXT_STACK_LENGTH (128 * 1024)
#define STACK_ALIGNMENT_BYTES     (8)
#define ASSERT(x)                 assert((x))
#define ARRAY_LENGTH(array)       (sizeof((array))/sizeof((array)[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This container stores the exit information for a NuttX task */

typedef struct exit_data_context_s
{
  ucontext_t *parent_ucontext;    /* We jump to this context after we release
                                   * the resources for the task that wants to
                                   * exit.
                                   */
  ucontext_t *current_ucontext;
  void *ucontext_sp;
  int cpu_index;
} exit_data_context_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This context is used on the exit path for the initial task */

static ucontext_t g_uctx_main;

/* We use an alternate ucontext for the task exit point to tear down
 * allocated resources. On the task exit we jump to this exit context
 * and we free the stack memory used by ucontext.
 */

#ifdef CONFIG_SMP
static exit_data_context_t g_task_exit_details[CONFIG_SMP_NCPUS];
static ucontext_t g_task_exit_context[CONFIG_SMP_NCPUS];
static uint8_t
  g_task_exit_context_stack[CONFIG_SMP_NCPUS][MIN_UCONTEXT_STACK_LENGTH];
#else
static exit_data_context_t g_task_exit_details[1];
static ucontext_t g_task_exit_context[1];
static uint8_t g_task_exit_context_stack[1][MIN_UCONTEXT_STACK_LENGTH];
#endif

/****************************************************************************
 * Public Prototype
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_index(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_exit_point
 *
 * Description:
 *   This function is used to release the allocated stack memory for a task
 *   on it's exit path.
 *
 * Assumptions:
 *   This function does not return.
 *
 ****************************************************************************/

static void task_exit_point(void)
{
  for (; ; )
    {
      int cpu_id = 0;

#ifdef CONFIG_SMP
      cpu_id = up_cpu_index();
#endif
      if (g_task_exit_details[cpu_id].ucontext_sp)
        {
          /* Free stack memory used by the ucontext structure */

          free(g_task_exit_details[cpu_id].current_ucontext);
          free(g_task_exit_details[cpu_id].ucontext_sp);

          /* Activate the parent ucontext */

          setcontext(g_task_exit_details[cpu_id].parent_ucontext);
        }
    }
}

/****************************************************************************
 * Name: up_setup_exit_context
 *
 * Description:
 *   Creates new exit context for tasks. In a SMP configuration every CPU
 *   has a different exit context.
 *
 ****************************************************************************/

static void up_setup_exit_context(int cpu)
{
  ucontext_t *exit_context = &g_task_exit_context[cpu];
  int ret = getcontext(exit_context);
  ASSERT(ret >= 0);

  uint8_t *sp = &g_task_exit_context_stack[cpu][0];

  uint64_t align_offset = STACK_ALIGNMENT_BYTES - ((uint64_t)sp %
      STACK_ALIGNMENT_BYTES);

  uint8_t *aligned_stack = sp + align_offset;
  exit_context->uc_stack.ss_sp   = aligned_stack;
  exit_context->uc_stack.ss_size = MIN_UCONTEXT_STACK_LENGTH -
    align_offset;
  exit_context->uc_link          = &g_uctx_main;

  makecontext(exit_context, task_exit_point, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_create_context
 *
 * Description:
 *   Creates a new ucontext structure and initialize it.
 *
 * Input Parameters:
 *   ucontext_sp   - buffer where we store the ucontext stack pointer
 *   prev_ucontext - the parent ucontext used%i to return to when the
 *    execution of the current context ends.
 *   entry_point   - the entry point of the new context
 *
 * Return Value:
 *   A pointer to the ucontext structure or NULL in case something went
 *   wrong.
 *
 ****************************************************************************/

void *up_create_context(void **ucontext_sp, void *prev_ucontext,
    void (*entry_point)(void))
{
  ucontext_t *new_context = calloc(1, sizeof(ucontext_t));
  ASSERT(new_context != NULL);

  int ret = getcontext(new_context);
  ASSERT(ret >= 0);

  uint8_t *sp = calloc(1, MIN_UCONTEXT_STACK_LENGTH);
  ASSERT(sp != NULL);

  *((uintptr_t *)ucontext_sp) = (uintptr_t)sp;

  uint64_t align_offset = STACK_ALIGNMENT_BYTES - ((uint64_t)sp %
      STACK_ALIGNMENT_BYTES);

  uint8_t *aligned_stack = sp + align_offset;
  new_context->uc_stack.ss_sp   = aligned_stack;
  new_context->uc_stack.ss_size = MIN_UCONTEXT_STACK_LENGTH - align_offset;
  new_context->uc_link          = prev_ucontext == NULL ? &g_uctx_main :
    prev_ucontext;

  makecontext(new_context, entry_point, 0);

  return new_context;
}

/****************************************************************************
 * Name: up_destroy_context
 *
 * Description:
 *   This function saves the ucontext for the current task and the parent
 *   task in a container and then activates the exit context. This mechanism
 *   is used to free the stack memory that we are currently executing on.
 *
 * Input Parameters:
 *   current_ucontext - pointer to the current active ucontext structure
 *   ucontext_sp      - pointer to the ucontext stack
 *   parent_ucontext  - the next context that we want to jump to
 *   cpu_id           - the CPU identificator
 *
 * Assumptions:
 *   This function does not return.
 *
 ****************************************************************************/

void up_destroy_context(void *current_ucontext, void *ucontext_sp,
    void *parent_ucontext, int cpu_id)
{
  g_task_exit_details[cpu_id].parent_ucontext  = parent_ucontext;
  g_task_exit_details[cpu_id].current_ucontext = current_ucontext;
  g_task_exit_details[cpu_id].cpu_index        = cpu_id;
  g_task_exit_details[cpu_id].ucontext_sp      = ucontext_sp;

  up_setup_exit_context(cpu_id);

  setcontext(&g_task_exit_context[cpu_id]);
}

/****************************************************************************
 * Name: up_swap_context
 *
 * Description:
 *   Save the current context in the old_ucontext and activate the
 *   context from activate_ucontext.
 *
 * Input Parameters:
 *   old_ucontext       - place where we store the current context
 *   activate_ucontext  - context that we will activate after function
 *     invocation
 *
 ****************************************************************************/

void up_swap_context(void *old_ucontext, void *activate_ucontext)
{
  swapcontext(old_ucontext, activate_ucontext);
}

/****************************************************************************
 * Name: up_set_context
 *
 * Description:
 *   Set the current context to the specified ucontext
 *
 * Input Parameters:
 *   current_context - the current context
 *
 ****************************************************************************/

void up_set_context(void *current_context)
{
  setcontext(current_context);
}
