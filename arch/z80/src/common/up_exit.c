/****************************************************************************
 * common/up_exit.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <chip/chip.h>

#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_DUMP_ON_EXIT
#include <nuttx/fs.h>
#endif

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _up_dumponexit
 *
 * Description:
 *   Dump the state of all tasks whenever on task exits.  This
 *   is debug instrumentation that was added to check file-
 *   related reference counting but could be useful again
 *   sometime in the future.
 *
 ****************************************************************************/

#if defined(CONFIG_DUMP_ON_EXIT) && defined(CONFIG_DEBUG)
static void _up_dumponexit(FAR _TCB *tcb, FAR void *arg)
{
#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NFILE_STREAMS > 0
  int i;
#endif

  dbg("  TCB=%p name=%s\n", tcb, tcb->argv[0]);

#if CONFIG_NFILE_DESCRIPTORS > 0
  if (tcb->filelist)
    {
      lldbg("    filelist refcount=%d\n",
            tcb->filelist->fl_crefs);

      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          struct inode *inode = tcb->filelist->fl_files[i].f_inode;
          if (inode)
            {
              lldbg("      fd=%d refcount=%d\n",
                    i, inode->i_crefs);
            }
        }
    }
#endif

#if CONFIG_NFILE_STREAMS > 0
  if (tcb->streams)
    {
      lldbg("    streamlist refcount=%d\n",
            tcb->streams->sl_crefs);

      for (i = 0; i < CONFIG_NFILE_STREAMS; i++)
        {
          struct file_struct *filep = &tcb->streams->sl_streams[i];
          if (filep->fs_filedes >= 0)
            {
#if CONFIG_STDIO_BUFFER_SIZE > 0
              lldbg("      fd=%d nbytes=%d\n",
                    filep->fs_filedes,
                    filep->fs_bufpos - filep->fs_bufstart);
#else
              lldbg("      fd=%d\n", filep->fs_filedes);
#endif
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
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete().
 *
 ****************************************************************************/

void _exit(int status)
{
  FAR _TCB* tcb = (FAR _TCB*)g_readytorun.head;

  /* Disable interrupts.  Interrupts will remain disabled until
   * the new task is resumed below.
   */

  (void)irqsave();

  lldbg("TCB=%p exitting\n", tcb);

#if defined(CONFIG_DUMP_ON_EXIT) && defined(CONFIG_DEBUG)
  lldbg("Other tasks:\n");
  sched_foreach(_up_dumponexit, NULL);
#endif

  /* Remove the tcb task from the ready-to-run list.  We can
   * ignore the return value because we know that a context
   * switch is needed.
   */

  (void)sched_removereadytorun(tcb);

  /* We are not in a bad stack-- the head of the ready to run task list
   * does not correspond to the thread that is running.  Disabling pre-
   * emption on this TCB should be enough to keep things stable.
   */

  sched_lock();

  /* Move the TCB to the specified blocked task list and delete it */

  sched_addblocked(tcb, TSTATE_TASK_INACTIVE);
  task_delete(tcb->pid);

  /* If there are any pending tasks, then add them to the g_readytorun
   * task list now
   */

  if (g_pendingtasks.head)
    {
      (void)sched_mergepending();
    }

  /* Now calling sched_unlock() should have no effect */

  sched_unlock();

  /* Now, perform the context switch to the new ready-to-run task at the
   * head of the list.
   */

  tcb = (FAR _TCB*)g_readytorun.head;
  lldbg("New Active Task TCB=%p\n", tcb);

  /* Then switch contexts */

  RESTORE_USERCONTEXT(tcb);
}

