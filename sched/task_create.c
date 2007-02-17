/************************************************************
 * task_create.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <sched.h>
#include <string.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/os_external.h>
#include "os_internal.h"

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

static void     task_start(void);
static STATUS   task_assignpid(_TCB* tcb);

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: task_start
 *
 * Description:
 *   This function is the low level entry point
 *   into the main thread of execution of a task.  It receives
 *   initial control when the task is started and calls main
 *   entry point of the newly started task.
 *
 * Inputs:
 *   None
 *
 * Return:
 *   None
 *
 ************************************************************/

static void task_start(void)
{
  _TCB *tcb = (_TCB*)g_readytorun.head;
  int argc;

  /* Count how many non-null arguments we are passing */

  for (argc = 1; argc <= NUM_TASK_ARGS; argc++)
    {
       /* The first non-null argument terminates the list */

       if (!tcb->argv[argc])
         {
           break;
         }
    }

  /* Call the 'main' entry point passing argc and argv.  If/when
   * the task returns,  */

  exit(tcb->entry.main(argc, tcb->argv));
}

/************************************************************
 * Name: task_assignpid
 *
 * Description:
 *   This function assigns the next unique task ID to a task.
 *
 * Inputs:
 *   tcb - TCB of task
 *
 * Return:
 *   OK on success; ERROR on failure (errno is not set)
 *
 ************************************************************/

static STATUS task_assignpid(_TCB *tcb)
{
  pid_t next_pid;
  int   hash_ndx;
  int   tries = 0;

  /* Disable pre-emption.  This should provide sufficient protection
   * for the following operation.
   */

  (void)sched_lock();

  /* We'll try every allowable pid */

  for (tries = 0; tries < MAX_TASKS_ALLOWED; tries++)
    {
      /* Get the next process ID candidate */

      next_pid = ++g_lastpid;

      /* Verify that the next_pid is in the valid range */

      if (next_pid <= 0)
        {
          g_lastpid = 1;
          next_pid  = 1;
        }

      /* Get the hash_ndx associated with the next_pid */

      hash_ndx = PIDHASH(next_pid);

      /* Check if there is a (potential) duplicate of this pid */

      if (!g_pidhash[hash_ndx].tcb)
        {
          g_pidhash[hash_ndx].tcb = tcb;
          g_pidhash[hash_ndx].pid = next_pid;
          tcb->pid = next_pid;
          (void)sched_unlock();
          return OK;
        }
    }

  /* If we get here, then the g_pidhash[] table is completely full.
   * We cannot allow another task to be started.
   */

  (void)sched_unlock();
  return ERROR;
}

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Name: _task_init and task_init
 *
 * Description:
 *   These functions initializes a Task Control Block (TCB)
 *   in preparation for starting a new thread.  _task_init()
 *   is an internal version of the function that has some
 *   additional control arguments and task_init() is a wrapper
 *   function that creates a VxWorks-like user API.
 *   task_init() is, otherwise, not used by the OS.
 *
 *   _task_init() is called from task_init() and task_start().\
 *   It is also called from pthread_create() to create a 
 *   a pthread (distinguished by the pthread argument).
 *
 *   Unlike task_create(), task_init() does not activate the
 *   task.  This must be done by calling task_activate()
 *   afterward.
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task (not used)
 *   priority   - Priority of the new task
 *   entry      - Entry point of a new task
 *   main       - Application start point of the new task
 *   pthread    - TRUE is the task emulates pthread behavior
 *   arg1-4     - Four required task arguments to pass to
 *                the task when it is started.
 *
 * Return Value:
 *  OK on success; ERROR on failure.
 *
 *  This function can only failure is it is unable to assign
 *  a new, unique task ID to the TCB (errno is not set).
 *
 ************************************************************/

STATUS _task_init(_TCB *tcb, char *name, int priority,
                  start_t start, main_t main, boolean pthread,
                  char *arg1, char *arg2, char *arg3, char *arg4)
{
  STATUS ret;

  /* Assign a unique task ID to the task. */

  ret = task_assignpid(tcb);
  if (ret == OK)
    {
      /* Save task priority and entry point in the TCB */

      tcb->init_priority  = (ubyte)priority;
      tcb->sched_priority = (ubyte)priority;
      tcb->start          = start;
      tcb->entry.main     = main;

#if CONFIG_TASK_NAME_SIZE > 0
      /* Give a name to the unnamed threads */

      if (!name)
        {
          name = "no name";
        }

      /* copy the name into the TCB */

      strncpy(tcb->name, name, CONFIG_TASK_NAME_SIZE);
#endif /* CONFIG_TASK_NAME_SIZE */

      /* Save the arguments in the TCB */

#if CONFIG_TASK_NAME_SIZE > 0
      tcb->argv[0] = tcb->name;
#else
      tcb->argv[0] = "no name";
#endif

      /* For pthreads, args are strictly pass-by-value; the char*
       * arguments wrap some unknown value cast to char*.  However,
       * for tasks, the life of the argument must be as long as
       * the life of the task and the arguments must be strings.
       * So for tasks, we have to to dup the strings.
       */

      if (!pthread)
        {
          /* The first NULL argument terminates the list of 
           * arguments.
           */

          if (arg1)
            {
              tcb->argv[1] = strdup(arg1);
              if (arg2)
                {
                  tcb->argv[2] = strdup(arg2);
                  if (arg3)
                    {
                      tcb->argv[3] = strdup(arg3);
                      if (arg4)
                        {
                          tcb->argv[4] = strdup(arg4);
                        }
                    }
                }
            }
        }
      else
        {
          /* Mark this task as a pthread */

          tcb->flags   |= TCB_FLAG_PTHREAD;

          /* And just copy the argument.  (For pthreads, there
           * is really only a single argument, arg1).
           */

          tcb->argv[1]  = arg1;
          tcb->argv[2]  = arg2;
          tcb->argv[3]  = arg3;
          tcb->argv[4]  = arg4;
        }

      /* Initialize other (non-zero) elements of the TCB */

      tcb->sigprocmask  = ALL_SIGNAL_SET;
      tcb->task_state    = TSTATE_TASK_INVALID;

      /* Initialize the processor-specific portion of the TCB */

      up_initial_state(tcb);

      /* Add the task to the inactive task list */

      sched_lock();
      dq_addfirst((dq_entry_t*)tcb, &g_inactivetasks);
      tcb->task_state = TSTATE_TASK_INACTIVE;
      sched_unlock();
    }

 return ret;
}

/************************************************************
 * Name: _task_init and task_init
 *
 * Description:
 *   This is a wrapper around the internal _task_init() that
 *   provides a VxWorks-like API.  See _task_init() for
 *   further information.
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task (not used)
 *   priority   - Priority of the new task
 *   stack      - start of the pre-allocated stack
 *   stack_size - size (in bytes) of the stack allocated
 *   entry      - Application start point of the new task
 *   arg1-4     - Four required task arguments to pass to
 *                the task when it is started.
 *
 * Return Value:
 *   see _task_init()
 *
 ************************************************************/

STATUS task_init(_TCB *tcb, char *name, int priority,
                 uint32 *stack, uint32 stack_size, main_t entry,
                 char *arg1, char *arg2, char *arg3, char *arg4)
{
  up_use_stack(tcb, stack, stack_size);
  return _task_init(tcb, name, priority, task_start, entry,
                    FALSE, arg1, arg2, arg3, arg4);
}

/************************************************************
 * Name: task_activate
 *
 * Description:
 *   This function activates tasks initialized by _task_init().
 *   Without activation, a task is ineligible for execution
 *   by the scheduler.
 *
 * Input Parameters:
 *   tcb - The TCB for the task for the task (same as the
 *         task_init argument.
 *
 * Return Value:
 *  Always returns OK
 *
 ************************************************************/

STATUS task_activate(_TCB *tcb)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
   uint32  savedState;
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
   savedState = irqsave();

   /* Check if this is really a re-start */

   if (tcb->task_state != TSTATE_TASK_INACTIVE)
     {
       /* Inform the instrumentation layer that the task
        * has stopped
        */

       sched_note_stop(tcb);
     }

   /* Inform the instrumentation layer that the task
    * has started
    */

   sched_note_start(tcb);
   irqrestore(savedState);
#endif

   up_unblock_task(tcb);
   return OK;
}

/************************************************************
 * Name: task_create
 *
 * Description:
 *   This function creates and activates a new task with a
 *   specified priority and returns its system-assigned ID.
 *
 *   The entry address entry is the address of the "main"
 *   function of the task.  This function will be called once
 *   the C environment has been set up.  The specified
 *   function will be called with four arguments.  Should
 *   the specified routine return, a call to exit() will
 *   automatically be made.
 *
 *   Note that four (and only four) arguments must be passed for
 *   the spawned functions.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg*       - Ten required task arguments to pass to func
 *
 * Return Value:
 *   Returns the non-zero process ID of the new task or
 *   ERROR if memory is insufficient or the task cannot be
 *   created (errno is not set).
 *
 ************************************************************/

int task_create(char *name, int priority,
                int stack_size, main_t entry,
                char *arg1, char *arg2, char *arg3, char *arg4)
{
  _TCB *tcb;
  STATUS status;
  pid_t pid;

  /* Allocate a TCB for the new task. */

  tcb = (_TCB*)kzmalloc(sizeof(_TCB));
  if (!tcb)
    {
      *get_errno_ptr() = ENOMEM;
      return ERROR;
    }

  /* Associate file descriptors with the new task */

  if (sched_setuptaskfiles(tcb) != OK)
    {
      sched_releasetcb(tcb);
      return ERROR;
    }

  /* Allocate the stack for the TCB */

  status = up_create_stack(tcb, stack_size);
  if (status != OK)
    {
      sched_releasetcb(tcb);
      return ERROR;
    }

   /* Initialize the task control block */

   status = _task_init(tcb, name, priority, task_start, entry,
                       FALSE, arg1, arg2, arg3, arg4);
   if (status != OK)
     {
       sched_releasetcb(tcb);
       return ERROR;
     }

   /* Get the assigned pid before we start the task */

   pid = (int)tcb->pid;

   /* Activate the task */

   status = task_activate(tcb);
   if (status != OK)
    {
      dq_rem((dq_entry_t*)tcb, &g_inactivetasks);
      sched_releasetcb(tcb);
      return ERROR;
    }

   return pid;
}
