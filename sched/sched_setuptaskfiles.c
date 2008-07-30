/****************************************************************************
 * sched/sched_setuptaskfiles.c
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

#include <sched.h>
#include <errno.h>

#include <nuttx/fs.h>
#include <nuttx/net.h>

#include "os_internal.h"

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  sched_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Parameters:
 *   tcb - tcb of the new task.
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setuptaskfiles(FAR _TCB *tcb)
{
#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR _TCB *rtcb = (FAR _TCB*)g_readytorun.head;
  int i;
#endif /* CONFIG_NFILE_DESCRIPTORS > 0 */
  int ret = OK;

#if CONFIG_NFILE_DESCRIPTORS > 0
  /* Allocate file descriptors for the TCB */

  tcb->filelist = files_alloclist();
  if (!tcb->filelist)
    {
      *get_errno_ptr() = ENOMEM;
       return ERROR;
    }
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  /* Allocate socket descriptors for the TCB */

  tcb->sockets = net_alloclist();
  if (!tcb->sockets)
    {
      *get_errno_ptr() = ENOMEM;
      return ERROR;
    }
#endif /* CONFIG_NSOCKET_DESCRIPTORS */

#if CONFIG_NFILE_DESCRIPTORS > 0
 /* Duplicate the first three file descriptors */

  if (rtcb->filelist)
    {
      for (i = 0; i < CONFIG_NFILE_DESCRIPTORS; i++)
        {
          /* Check if this file is opened */

          if (rtcb->filelist->fl_files[i].f_inode)
            {
              (void)files_dup(&rtcb->filelist->fl_files[i],
                              &tcb->filelist->fl_files[i]);
            }
        }
    }

#if CONFIG_NFILE_STREAMS > 0
  /* Allocate file streams for the TCB */

  ret = sched_setupstreams(tcb);
#endif /* CONFIG_NFILE_STREAMS */
#endif /* CONFIG_NFILE_DESCRIPTORS */
  return ret;
}

#endif /* CONFIG_NFILE_DESCRIPTORS || CONFIG_NSOCKET_DESCRIPTORS */
