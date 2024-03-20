/****************************************************************************
 * binfmt/binfmt_execmodule.c
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

#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <sched/sched.h>
#include <task/spawn.h>
#include <nuttx/spawn.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If C++ constructors are used, then CONFIG_SCHED_STARTHOOK must also be
 * selected be the start hook is used to schedule execution of the
 * constructors.
 */

#if defined(CONFIG_BINFMT_CONSTRUCTORS) && !defined(CONFIG_SCHED_STARTHOOK)
#  error "CONFIG_SCHED_STARTHOOK must be defined to use constructors"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_ctors
 *
 * Description:
 *   Execute C++ static constructors.  This function is registered as a
 *   start hook and runs on the thread of the newly created task before
 *   the new task's main function is called.
 *
 * Input Parameters:
 *   arg - Argument is instance of load state info structure cast to void *.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_CONSTRUCTORS
static void exec_ctors(FAR void *arg)
{
  FAR const struct binary_s *binp = (FAR const struct binary_s *)arg;
  binfmt_ctor_t *ctor = binp->ctors;
  int i;

  /* Execute each constructor */

  for (i = 0; i < binp->nctors; i++)
    {
      binfo("Calling ctor %d at %p\n", i, ctor);

      (*ctor)();
      ctor++;
    }
}
#endif

/****************************************************************************
 * Name: exec_swap
 *
 * Description:
 *   swap the pid of tasks, and reverse parent-child relationship.
 *
 * Input Parameters:
 *   ptcb  - parent task tcb.
 *   chtcb - child task tcb.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void exec_swap(FAR struct tcb_s *ptcb, FAR struct tcb_s *chtcb)
{
  int        pndx;
  int        chndx;
  pid_t      pid;
  irqstate_t flags;
#ifdef CONFIG_SCHED_HAVE_PARENT
#  ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *tg_children;
#  else
  uint16_t   tg_nchildren;
#  endif
#endif

  DEBUGASSERT(ptcb);
  DEBUGASSERT(chtcb);

  flags = enter_critical_section();

  pndx  = PIDHASH(ptcb->pid);
  chndx = PIDHASH(chtcb->pid);

  DEBUGASSERT(nxsched_pidhash()[pndx]);
  DEBUGASSERT(nxsched_pidhash()[chndx]);

  /* Exchange nxsched_pidhash() index */

  nxsched_pidhash()[pndx] = chtcb;
  nxsched_pidhash()[chndx] = ptcb;

  /* Exchange pid */

  pid = chtcb->pid;
  chtcb->pid = ptcb->pid;
  ptcb->pid = pid;

  /* Exchange group info. This will reverse parent-child relationship */

  pid = chtcb->group->tg_pid;
  chtcb->group->tg_pid = ptcb->group->tg_pid;
  ptcb->group->tg_pid = pid;

  pid = chtcb->group->tg_ppid;
  chtcb->group->tg_ppid = ptcb->group->tg_ppid;
  ptcb->group->tg_ppid = pid;

#ifdef CONFIG_SCHED_HAVE_PARENT
#  ifdef CONFIG_SCHED_CHILD_STATUS
  tg_children = chtcb->group->tg_children;
  chtcb->group->tg_children = ptcb->group->tg_children;
  ptcb->group->tg_children = tg_children;
#  else
  tg_nchildren = chtcb->group->tg_nchildren;
  chtcb->group->tg_nchildren = ptcb->group->tg_nchildren;
  ptcb->group->tg_nchildren = tg_nchildren;
#  endif
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int exec_module(FAR struct binary_s *binp,
                FAR const char *filename, FAR char * const *argv,
                FAR char * const *envp,
                FAR const posix_spawn_file_actions_t *actions,
                FAR const posix_spawnattr_t *attr,
                bool spawn)
{
  FAR struct task_tcb_s *tcb;
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  FAR struct arch_addrenv_s *addrenv = &binp->addrenv->addrenv;
  FAR void *vheap;
  char name[CONFIG_PATH_MAX];
#endif
  FAR void *stackaddr = NULL;
  pid_t pid;
  int ret;

  /* Sanity checking */

#ifdef CONFIG_DEBUG_FEATURES
  if (!binp || !binp->entrypt || binp->stacksize <= 0)
    {
      return -EINVAL;
    }
#endif

  binfo("Executing %s\n", filename);

  /* Allocate a TCB for the new task. */

  tcb = kmm_zalloc(sizeof(struct task_tcb_s));
  if (!tcb)
    {
      return -ENOMEM;
    }

  ret = binfmt_copyargv(&argv, argv);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Make a copy of the environment here */

  if (envp == NULL)
    {
      envp = environ;
    }

  ret = binfmt_copyenv(&envp, envp);
  if (ret < 0)
    {
      goto errout_with_args;
    }

  ret = binfmt_copyactions(&actions, actions);
  if (ret < 0)
    {
      goto errout_with_envp;
    }

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* If there is no argument vector, the process name must be copied here */

  if (argv == NULL)
    {
      strlcpy(name, filename, CONFIG_PATH_MAX);
      filename = name;
    }

  /* Instantiate the address environment containing the user heap */

  ret = addrenv_select(binp->addrenv, &binp->oldenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_select() failed: %d\n", ret);
      goto errout_with_actions;
    }

  ret = up_addrenv_vheap(addrenv, &vheap);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_vheap() failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  binfo("Initialize the user heap (heapsize=%zu)\n",
        up_addrenv_heapsize(addrenv));
  umm_initialize(vheap, up_addrenv_heapsize(addrenv));
#endif

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_ARCH_KERNEL_STACK)
  /* Allocate the kernel stack */

  ret = up_addrenv_kstackalloc(&tcb->cmn);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_kstackalloc() failed: %d\n", ret);
      goto errout_with_addrenv;
    }
#endif

  /* Note that tcb->cmn.flags are not modified.  0=normal task */

  /* tcb->cmn.flags |= TCB_FLAG_TTYPE_TASK; */

  tcb->cmn.flags |= TCB_FLAG_FREE_TCB;

  /* Initialize the task */

#ifndef CONFIG_BUILD_KERNEL
  stackaddr = binp->stackaddr;
#endif

  if (argv && argv[0])
    {
      ret = nxtask_init(tcb, argv[0], binp->priority, stackaddr,
                        binp->stacksize, binp->entrypt, &argv[1],
                        envp, actions);
    }
  else
    {
      ret = nxtask_init(tcb, filename, binp->priority, stackaddr,
                        binp->stacksize, binp->entrypt, argv,
                        envp, actions);
    }

  if (ret < 0)
    {
      berr("nxtask_init() failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* The copied argv and envp can now be released */

  binfmt_freeactions(actions);
  binfmt_freeargv(argv);
  binfmt_freeenv(envp);

#ifdef CONFIG_PIC
  /* Add the D-Space address as the PIC base address.  By convention, this
   * must be the first allocated address space.
   */

  tcb->cmn.dspace = binp->alloc[0];

  /* Re-initialize the task's initial state to account for the new PIC base */

  up_initial_state(&tcb->cmn);
#endif

#ifdef CONFIG_ARCH_ADDRENV
  /* Attach the address environment to the new task */

  ret = addrenv_attach((FAR struct tcb_s *)tcb, binp->addrenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_attach() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }
#endif

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  /* Setup a start hook that will execute all of the C++ static constructors
   * on the newly created thread.  The struct binary_s must persist at least
   * until the new task has been started.
   */

  if (binp->nctors > 0)
    {
      nxtask_starthook(tcb, exec_ctors, binp);
    }
#endif

#ifdef CONFIG_SCHED_USER_IDENTITY
  if (binp->mode & S_ISUID)
    {
      tcb->cmn.group->tg_euid = binp->uid;
    }

  if (binp->mode & S_ISGID)
    {
      tcb->cmn.group->tg_egid = binp->gid;
    }
#endif

  if (!spawn)
    {
      exec_swap(this_task(), (FAR struct tcb_s *)tcb);
    }

  /* Get the assigned pid before we start the task */

  pid = tcb->cmn.pid;

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  /* Restore the address environment of the caller */

  ret = addrenv_restore(binp->oldenv);
  if (ret < 0)
    {
      berr("ERROR: addrenv_restore() failed: %d\n", ret);
      goto errout_with_tcbinit;
    }
#endif

  /* Set the attributes */

  if (attr)
    {
      ret = spawn_execattrs(pid, attr);
      if (ret < 0)
        {
          goto errout_with_tcbinit;
        }
    }

  /* Then activate the task at the provided priority */

  nxtask_activate((FAR struct tcb_s *)tcb);

  return pid;

errout_with_tcbinit:
#ifndef CONFIG_BUILD_KERNEL
  if (binp->stackaddr != NULL)
    {
      tcb->cmn.stack_alloc_ptr = NULL;
    }
#endif

  nxtask_uninit(tcb);
  return ret;

errout_with_addrenv:
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  addrenv_restore(binp->oldenv);
errout_with_actions:
  binfmt_freeactions(actions);
#endif
errout_with_envp:
  binfmt_freeenv(envp);
errout_with_args:
  binfmt_freeargv(argv);
errout_with_tcb:
  kmm_free(tcb);
  return ret;
}

#endif /* CONFIG_BINFMT_DISABLE */
