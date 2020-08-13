/****************************************************************************
 * arch/z16/src/common/z16_exit.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#ifdef CONFIG_DUMP_ON_EXIT
#  include <nuttx/fs/fs.h>
#endif

#include "chip.h"
#include "task/task.h"
#include "sched/sched.h"
#include "z16_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DEBUG_SCHED_INFO
#  undef CONFIG_DUMP_ON_EXIT
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _z16_dumponexit
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This is debug
 *   instrumentation that was added to check file-related reference counting
 *   but could be useful again sometime in the future.
 *
 ****************************************************************************/

#ifdef CONFIG_DUMP_ON_EXIT
static void _z16_dumponexit(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct filelist *filelist;
#ifdef CONFIG_FILE_STREAM
  FAR struct file_struct *filep;
#endif
  int i;

  sinfo("  TCB=%p name=%s\n", tcb, tcb->argv[0]);
  sinfo("    priority=%d state=%d\n", tcb->sched_priority, tcb->task_state);

  filelist = tcb->group->tg_filelist;
  for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
    {
      struct inode *inode = filelist->fl_files[i].f_inode;
      if (inode)
        {
          sinfo("      fd=%d refcount=%d\n",
                i, inode->i_crefs);
        }
    }

#ifdef CONFIG_FILE_STREAM
  filep = tcb->group->tg_streamlist->sl_head;
  for (; filep != NULL; filep = filep->fs_next)
    {
      if (filep->fs_fd >= 0)
        {
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
          if (filep->fs_bufstart != NULL)
            {
              sinfo("      fd=%d nbytes=%d\n",
                    filep->fs_fd,
                    filep->fs_bufpos - filep->fs_bufstart);
            }
          else
#endif
            {
              sinfo("      fd=%d\n", filep->fs_fd);
            }
        }
    }
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 ****************************************************************************/

void up_exit(int status)
{
  FAR struct tcb_s *tcb = this_task();

  /* Make sure that we are in a critical section with local interrupts.
   * The IRQ state will be restored when the next task is started.
   */

  enter_critical_section();

  sinfo("TCB=%p exiting\n", tcb);

#ifdef CONFIG_DUMP_ON_EXIT
  sinfo("Other tasks:\n");
  nxsched_foreach(_z16_dumponexit, NULL);
#endif

  /* Destroy the task at the head of the ready to run list. */

  nxtask_exit();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = this_task();
  sinfo("New Active Task TCB=%p\n", tcb);

  /* Then switch contexts */

  RESTORE_USERCONTEXT(tcb);
}
