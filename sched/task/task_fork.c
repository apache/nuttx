/****************************************************************************
 * sched/task/task_fork.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/wait.h>
#include <stdint.h>
#include <sched.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>

#include "sched/sched.h"
#include "environ/environ.h"
#include "group/group.h"
#include "task/task.h"
#include "tls/tls.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
static void fork_inherit_stack(FAR struct tcb_s *parent,
                               FAR struct tcb_s *child);
static void fork_inherit_tls(FAR struct tcb_s *child);
static void fork_restore_parent_env(void);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
/****************************************************************************
 * Name: fork_inherit_stack
 *
 * Description:
 *   Give the fork() child the parent's stack at the parent's virtual
 *   address rather than a relocated copy.  The child's address environment
 *   is a duplicate, so the parent's stack is already there -- same contents,
 *   same address, its own pages -- and nothing needs allocating or copying.
 *
 *   A relocated stack would break plain C:  a pointer to a local taken
 *   before the fork would name the parent's copy, not the child's live
 *   object.
 *
 *   TCB_FLAG_FREE_STACK is left clear:  the stack belongs to the duplicated
 *   image and is released with it, so up_release_stack() must not free it.
 *
 * Input Parameters:
 *   parent - The parent task's TCB
 *   child  - The child task's TCB
 *
 ****************************************************************************/

static void fork_inherit_stack(FAR struct tcb_s *parent,
                               FAR struct tcb_s *child)
{
  child->stack_alloc_ptr = parent->stack_alloc_ptr;
  child->stack_base_ptr  = parent->stack_base_ptr;
  child->adj_stack_size  = parent->adj_stack_size;
  child->flags          &= ~TCB_FLAG_FREE_STACK;
}

/****************************************************************************
 * Name: fork_inherit_tls
 *
 * Description:
 *   Retarget the thread-local storage the fork() child inherited.
 *
 *   tls_dup_info() cannot be used:  it carves a fresh TLS block off the
 *   stack, which on an inherited stack would carve a second one and shift
 *   stack_base_ptr away from the parent's.  The child's copy is already in
 *   place, so only the fields naming the task itself need correcting.
 *
 *   The write lands in user memory at an address the parent also occupies,
 *   so the child's address environment must be current for it -- otherwise
 *   the parent's own TLS is what gets modified.
 *
 * Input Parameters:
 *   child - The child task's TCB
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static void fork_inherit_tls(FAR struct tcb_s *child)
{
  FAR struct tls_info_s *info = (FAR struct tls_info_s *)
                                child->stack_alloc_ptr;

  info->tl_task = child->group->tg_info;
  info->tl_tid  = child->pid;
}

/****************************************************************************
 * Name: fork_restore_parent_env
 *
 * Description:
 *   Undo the addrenv_select() that nxtask_setup_fork() made on the child's
 *   behalf, putting the caller back in its own address environment.  The
 *   environment to go back to does not have to be remembered:  the caller is
 *   the parent, and what was current before was the parent's own.
 *
 ****************************************************************************/

static void fork_restore_parent_env(void)
{
  addrenv_restore(this_task()->addrenv_own);
}
#endif /* CONFIG_ARCH_ADDRENV && CONFIG_ARCH_HAVE_FORK */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_setup_fork
 *
 * Description:
 *   Allocate and initialize the child task's TCB.  This is one step in the
 *   sequence common to vfork() and fork(); see the comment above the
 *   prototype in include/nuttx/sched.h for the whole sequence and for what
 *   the two primitives mean.
 *
 *   Exactly two things depend on `vfork':
 *
 *   - the address environment:  vfork() joins the parent's, fork()
 *     duplicates it.
 *   - the stack:  a vfork() child gets its own, which the architecture code
 *     fills with a relocated copy; a fork() child inherits the parent's
 *     address (fork_inherit_stack()).
 *
 * Input Parameters:
 *   retaddr - Address at which the child resumes
 *   vfork   - true for vfork(), false for fork()
 *
 * Returned Value:
 *   Upon successful completion, nxtask_setup_fork() returns a pointer to
 *   newly allocated and initialized child task's TCB.  NULL is returned
 *   on any failure and the errno is set appropriately.
 *
 ****************************************************************************/

FAR struct tcb_s *nxtask_setup_fork(start_t retaddr, bool vfork)
{
  FAR struct tcb_s *ptcb = this_task();
  FAR struct tcb_s *parent;
  FAR struct tcb_s *child;
  FAR char **argv;
  size_t stack_size;
  uint8_t ttype;
  int priority;
  int ret;

  DEBUGASSERT(retaddr != NULL);

  /* Get the type of the fork'ed task (kernel or user) */

  if ((ptcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      /* Fork'ed from a kernel thread */

      ttype = TCB_FLAG_TTYPE_KERNEL;
      parent = ptcb;
    }
  else
    {
      /* Fork'ed from a user task or pthread */

      ttype = TCB_FLAG_TTYPE_TASK;
      if ((ptcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_TASK)
        {
          parent = ptcb;
        }
      else
        {
          parent = nxsched_get_tcb(ptcb->group->tg_pid);
          if (parent == NULL)
            {
              ret = -ENOENT;
              goto errout;
            }
        }
    }

  /* Allocate a TCB for the child task. */

  child = kmm_zalloc(sizeof(struct tcb_s));
  if (!child)
    {
      serr("ERROR: Failed to allocate TCB\n");
      ret = -ENOMEM;
      goto errout;
    }

  child->flags |= TCB_FLAG_FREE_TCB;

  /* Initialize the task join */

  nxtask_joininit(child);

  /* Allocate a new task group with the same privileges as the parent */

  ret = group_allocate(child, ttype);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

#if defined(CONFIG_ARCH_ADDRENV)
  if (ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      if (vfork)
        {
          /* vfork():  join the parent address environment, exactly as
           * pthread_create() does.  The child shares .data, .bss and the
           * heap.
           */

          ret = addrenv_join(parent, child);
        }
#ifdef CONFIG_ARCH_HAVE_FORK
      else
        {
          /* POSIX fork():  duplicate the parent's address environment now,
           * before anything else is set up.  The duplicate holds a copy of
           * the parent's contents -- including its stack -- at the parent's
           * virtual addresses, which is what lets the child go on to inherit
           * the stack address rather than be given a relocated copy.  See
           * fork_inherit_stack().
           */

          ret = addrenv_fork(parent, child);
          if (ret >= 0)
            {
              /* Make the child's address environment current for the rest of
               * the setup, and for the architecture code that runs after it.
               *
               * From here on, everything written on the child's behalf
               * has to land in the child's image rather than the parent's,
               * because
               * the two occupy the same virtual addresses:  its thread-local
               * storage, and -- on architectures that keep the register save
               * area on the user stack rather than on a kernel stack -- the
               * register context the child is resumed from.  Writing those
               * under the parent's environment corrupts the parent and
               * leaves the child reading whatever the snapshot happened to
               * contain.
               *
               * Reads are unaffected:  everything the setup reads from the
               * parent -- environ, the argument vector -- is legible at the
               * same address in the child, precisely because it is a copy.
               *
               * nxtask_start_fork() puts the parent's environment back.
               */

              FAR struct addrenv_s *oldenv;

              ret = addrenv_select(child->addrenv_own, &oldenv);
            }
        }
#else
      /* An address environment without ARCH_HAVE_FORK -- a protected build
       * over an MMU, for instance.  There is an address environment to join,
       * but no POSIX fork() to duplicate it for, so the branch above is not
       * compiled and `vfork' is always true here.
       */

      DEBUGASSERT(vfork);
#endif

      if (ret < 0)
        {
          goto errout_with_tcb;
        }
    }
#else
  /* Without address environments there is only one address space, so
   * everything except the stack is shared no matter which primitive was
   * called.  POSIX fork() cannot be provided at all, and CONFIG_ARCH_HAVE_
   * FORK is not selected, so `vfork' is always true here.
   */

  DEBUGASSERT(vfork);
#endif

  /* Duplicate the parent tasks environment */

  ret = env_dup(child->group, environ);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(child, NULL, false);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Set the task name */

  argv = nxsched_get_stackargs(parent);
  nxtask_setup_name(child, argv[0]);

  /* Allocate the stack for the TCB, or inherit the parent's */

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
  if (!vfork && ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      /* The child's copy of the parent's stack is already in place, at the
       * parent's address, courtesy of the duplication above.
       */

      fork_inherit_stack(parent, child);
      ret = OK;
    }
  else
#endif
    {
      stack_size = (uintptr_t)ptcb->stack_base_ptr -
                   (uintptr_t)ptcb->stack_alloc_ptr + ptcb->adj_stack_size;

      ret = up_create_stack(child, stack_size, ttype);
    }

  if (ret < OK)
    {
      goto errout_with_tcb;
    }

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)
  /* Allocate the kernel stack */

  if (ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      ret = up_addrenv_kstackalloc(child);
      if (ret < 0)
        {
          goto errout_with_tcb;
        }
    }
#endif

  /* Get the priority of the parent task */

#ifdef CONFIG_PRIORITY_INHERITANCE
  priority = ptcb->base_priority;   /* "Normal," unboosted priority */
#else
  priority = ptcb->sched_priority;  /* Current priority */
#endif

  /* Initialize the task control block.  This calls up_initial_state() */

  sinfo("Child priority=%d start=%p\n", priority, retaddr);
  ret = nxtask_setup_scheduler(child, priority, retaddr,
                               ptcb->entry.main, ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Set up thread local storage and the argument vector.
   *
   * A fork() child that inherited its stack already has both, byte for
   * byte, at the addresses the parent has them at -- they came across with
   * the rest of the image.  Re-creating them would carve fresh frames off a
   * stack that already contains them, moving stack_base_ptr away from the
   * parent's and undoing the inheritance.  Only the TLS fields that name
   * the task itself need correcting.
   */

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
  if (!vfork && ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      fork_inherit_tls(child);
    }
  else
#endif
    {
      ret = tls_dup_info(child, parent);
      if (ret < OK)
        {
          goto errout_with_tcb;
        }

      ret = nxtask_setup_stackargs(child, argv[0], &argv[1]);
      if (ret < OK)
        {
          goto errout_with_tcb;
        }
    }

  /* Now we have enough in place that we can join the group */

  group_initialize(child);
  sinfo("parent=%p, returning child=%p\n", parent, child);
  return child;

errout_with_tcb:
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
  /* Get back into the parent's address environment before unwinding.  If the
   * duplication above never happened this is the environment we are already
   * in, and addrenv_restore() is then a no-op.
   */

  if (!vfork && ttype != TCB_FLAG_TTYPE_KERNEL)
    {
      fork_restore_parent_env();
    }
#endif

  nxsched_release_tcb((FAR struct tcb_s *)child, ttype);
errout:
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: nxtask_start_fork
 *
 * Description:
 *   The last step of both primitives:  finish the child and run it.  The
 *   architecture-specific code calls this once it has built the child's
 *   register context and stack.
 *
 *   A vfork() additionally suspends the caller until the child calls _exit()
 *   or one of the exec family of functions.  The suspension lives here, in
 *   the kernel primitive, rather than in a libc waitpid() as it once did.
 *   Two things follow from that.  The parent is released when the child's
 *   TCB is torn down (see nxtask_resume_vfork()), which for an exec()ing
 *   child is immediately after exec_swap() has handed the child's pid to the
 *   program it loaded -- so the parent resumes at exec(), holding a pid that
 *   names the running program, as POSIX requires.  And vfork() no longer
 *   depends on CONFIG_SCHED_WAITPID.
 *
 * Input Parameters:
 *   child - The tcb_s struct instance created by nxtask_setup_fork()
 *   vfork - true for vfork(), false for fork()
 *
 * Returned Value:
 *   The process ID of the child, or ERROR on failure.
 *
 ****************************************************************************/

pid_t nxtask_start_fork(FAR struct tcb_s *child, bool vfork)
{
#ifdef CONFIG_ARCH_HAVE_VFORK
  /* The rendezvous between the suspended parent and the child lives in this
   * frame:  the parent is blocked here for the whole lifetime of the child,
   * so the storage is alive exactly as long as it is needed, and no
   * allocation is required on a path that must not fail.
   */

  sem_t rel;
  int ret;
#endif
  pid_t pid;

  sinfo("Starting Child TCB=%p vfork=%d\n", child, vfork);
  DEBUGASSERT(child);

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
  /* The architecture code has finished writing the child's image, so put the
   * parent back in its own address environment.  See nxtask_setup_fork().
   */

  if (!vfork &&
      (child->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
    {
      fork_restore_parent_env();
    }
#endif

  /* Get the assigned pid before we start the task */

  pid = child->pid;

#ifdef CONFIG_ARCH_HAVE_VFORK
  if (vfork)
    {
      nxsem_init(&rel, 0, 0);
      child->vfork_rel = &rel;
    }
#endif

  /* Activate the task */

  nxtask_activate(child);

#ifdef CONFIG_ARCH_HAVE_VFORK
  if (vfork)
    {
      /* Wait for the child to _exit() or exec().  This is not a cancellation
       * point and must not be interrupted by a signal:  the child may be
       * running on our stack, so returning early would corrupt it.
       */

      do
        {
          ret = nxsem_wait_uninterruptible(&rel);
        }
      while (ret == -EINTR);

      nxsem_destroy(&rel);
    }
#endif

  return pid;
}

#ifdef CONFIG_ARCH_HAVE_VFORK
/****************************************************************************
 * Name: nxtask_resume_vfork
 *
 * Description:
 *   Release the vfork() parent suspended on this child, if there is one.
 *
 *   Called from nxsched_release_tcb(), the last point in the child's life,
 *   by which time an exec()ing child has already handed its pid to the
 *   program it loaded.  nxtask_abort_fork() reaches it too, so a fork that
 *   fails after the rendezvous also releases the parent.
 *
 * Input Parameters:
 *   child - The TCB being torn down
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_resume_vfork(FAR struct tcb_s *child)
{
  FAR sem_t *rel = child->vfork_rel;

  if (rel != NULL)
    {
      /* Clearing the pointer first is what makes this once-only. */

      child->vfork_rel = NULL;
      nxsem_post(rel);
    }
}
#endif /* CONFIG_ARCH_HAVE_VFORK */

/****************************************************************************
 * Name: nxtask_abort_fork
 *
 * Description:
 *   Recover from any errors after nxtask_setup_fork() was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_abort_fork(FAR struct tcb_s *child, int errcode)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_HAVE_FORK)
  /* A child holding an address environment of its own, rather than a
   * reference to the caller's, is a fork() child, and nxtask_setup_fork()
   * left that environment selected.  Get back into the parent's before
   * unwinding.  See nxtask_setup_fork().
   */

  if (child->addrenv_own != NULL &&
      child->addrenv_own != this_task()->addrenv_own)
    {
      fork_restore_parent_env();
    }
#endif

  /* The TCB was added to the active task list by nxtask_setup_scheduler() */

  dq_rem((FAR dq_entry_t *)child, list_inactivetasks());

  /* Release the TCB */

  nxsched_release_tcb(child, child->flags & TCB_FLAG_TTYPE_MASK);
  set_errno(errcode);
}
