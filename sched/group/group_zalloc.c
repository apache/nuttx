/****************************************************************************
 * sched/group/group_zalloc.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>

#include <string.h>

#include "group/group.h"

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_malloc
 *
 * Description:
 *   Allocate memory and clear appropriate for the group type.  If the
 *   memory is part of a privileged, then it should be allocated so
 *   that it is only accessed by privileged code;  Otherwise, it must be
 *   allocated so that it accessible by unprivileged code.
 *
 ****************************************************************************/

FAR void *group_zalloc(FAR struct task_group_s *group, size_t nbytes)
{
  FAR void *mem = group_malloc(group, nbytes);
  if (mem)
    {
      memset(mem, 0, nbytes);
    }

  return mem;
}

#endif  /* CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL) && CONFIG_MM_KERNEL_HEAP */
