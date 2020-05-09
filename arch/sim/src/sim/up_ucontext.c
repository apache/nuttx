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

/* The minimum ucontext size */

#define MIN_UCONTEXT_STACK_LENGTH (128 * 1024)
#define STACK_ALIGNMENT_BYTES     (8)
#define ASSERT(x) assert((x))
#define ARRAY_LENGTH(array) (sizeof((array))/sizeof((array)[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct exit_data_context_s
{
  ucontext_t *next_ucontext;
  ucontext_t *current_ucontext;
  void *ucontext_sp;
  int cpu_index;
} exit_data_context_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ucontext_t g_uctx_main;

#ifdef CONFIG_SMP
static exit_data_context_t g_task_exit_details[CONFIG_SMP_NCPUS];
static ucontext_t g_task_exit_context[CONFIG_SMP_NCPUS];
#else
static exit_data_context_t g_task_exit_details[1];
static ucontext_t g_task_exit_context[1];
#endif

static volatile bool g_is_exit_context_init;
static stack_t g_ss;

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
  while (1)
    {
      int cpu_id = 0;

#ifdef CONFIG_SMP
      cpu_id = up_cpu_index();
#endif
      getcontext(&g_task_exit_context[cpu_id]);

      if (g_task_exit_details[cpu_id].ucontext_sp)
        {
          free(g_task_exit_details[cpu_id].current_ucontext);
          free(g_task_exit_details[cpu_id].ucontext_sp);
          g_task_exit_details[cpu_id].ucontext_sp = NULL;
          setcontext(g_task_exit_details[cpu_id].next_ucontext);
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

static void up_setup_exit_context(void)
{
  for (int i = 0; i < ARRAY_LENGTH(g_task_exit_context); i++)
    {
      ucontext_t *exit_context = &g_task_exit_context[i];
      int ret = getcontext(exit_context);
      ASSERT(ret >= 0);

      uint8_t *sp = calloc(1, MIN_UCONTEXT_STACK_LENGTH);
      ASSERT(sp != NULL);

      uint64_t align_offset = STACK_ALIGNMENT_BYTES - ((uint64_t)sp %
          STACK_ALIGNMENT_BYTES);

      uint8_t *aligned_stack = sp + align_offset;
      exit_context->uc_stack.ss_sp   = aligned_stack;
      exit_context->uc_stack.ss_size = MIN_UCONTEXT_STACK_LENGTH -
        align_offset;
      exit_context->uc_link          = &g_uctx_main;

      makecontext(exit_context, task_exit_point, 0);
    }

  g_is_exit_context_init = true;
}

/****************************************************************************
 * Name: up_is_stack_downwards
 *
 * Description:
 *   This function returns true if the stack places elements downwards.
 *
 * Input Parameters:
 *   stack_arg - the address of a variable from the previous function stack
 *    frame
 *
 * Returned Value:
 *   True if the stack grows downwards otherwise false.
 *
 ****************************************************************************/

static bool up_is_stack_downwards(void *stack_arg)
{
  int stack_arg_local;
  return stack_arg > (void *)&stack_arg_local;
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

  if (!g_is_exit_context_init)
    up_setup_exit_context();

  return new_context;
}

/****************************************************************************
 * Name: up_destroy_context
 *
 * Description:
 *   This function releases the host memory for the context structure and the
 *   context stack and jumps to the next context specified as argument.
 *
 * Input Parameters:
 *   current_ucontext - pointer to the TCB context
 *   ucontext_sp      - pointer to the ucontext stack
 *   new_ucontext     - the next context that we want to jump to
 *   cpu_id           - the CPU identificator
 *
 * Assumptions:
 *   This function does not return. In order to free the stack memory that we
 *   are currently executing on, we jump to a new exit context.
 *
 ****************************************************************************/

void up_destroy_context(void *current_ucontext, void *ucontext_sp,
    void *next_ucontext, int cpu_id)
{
  g_task_exit_details[cpu_id].next_ucontext = next_ucontext;
  g_task_exit_details[cpu_id].current_ucontext = current_ucontext;
  g_task_exit_details[cpu_id].cpu_index     = cpu_id;
  g_task_exit_details[cpu_id].ucontext_sp   = ucontext_sp;

  setcontext(&g_task_exit_context[cpu_id]);
}

/****************************************************************************
 * Name: up_setup_args_context
 *
 * Description:
 *   This function specifies the entry point for the new context and sets up
 *   the arguments for the new task.
 *
 * Input Parameters:
 *   ucontext - pointer to the TCB context
 *   entry_point  - the entry point for the task
 *   argc         - the number of cmd arguments
 *   argv         - array of arguments for the new task
 *
 ****************************************************************************/

void up_setup_args_context(void *current_ucontext, void (*entry_point)(void),
    int argc, char *argv[])
{
  makecontext(current_ucontext, entry_point, 2, argc, argv);
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
 *     invocation.
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

/****************************************************************************
 * Name: up_cpu_simulated_interrupt
 *
 * Description:
 *   This function verifies if we are running on the signal handler stack.
 *
 * Returned Value:
 *   Return zero if we are not running on the signal stack otherwise return
 *   a non zero value.
 *
 ****************************************************************************/

int up_cpu_simulated_interrupt(void)
{
  uint8_t test_address;
  unsigned long verify_stack_arg = (unsigned long)&test_address;
  unsigned long stack_start = (unsigned long)g_ss.ss_sp;
  unsigned long stack_end;

  if (up_is_stack_downwards(&verify_stack_arg) == true)
    {
      stack_end = stack_start - g_ss.ss_size;
      return stack_start > verify_stack_arg && stack_end < verify_stack_arg;
    }
  else
    {
      stack_end = stack_start + g_ss.ss_size;
      return stack_start < verify_stack_arg && stack_end < verify_stack_arg;
    }
}

/****************************************************************************
 * Name: up_setup_sigstackt
 *
 * Description:
 *   This function sets up an alternative stack for the signals from the
 *   host heap memory.
 *
 ****************************************************************************/

void up_setup_sigstack(void)
{
  g_ss.ss_sp = malloc(SIGSTKSZ);
  ASSERT(g_ss.ss_sp != NULL);

  g_ss.ss_size  = SIGSTKSZ;
  g_ss.ss_flags = 0;

  int ret = sigaltstack(&g_ss, NULL);
  ASSERT(ret >= 0);
}
