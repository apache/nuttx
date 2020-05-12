/****************************************************************************
 * libs/libc/string/lib_psfa_init.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <spawn.h>
#include <nuttx/spawn.h>

#if defined(CONFIG_LIB_SYSCALL) && !defined(CONFIG_BUILD_KERNEL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_spawn
 *
 * Description:
 *   This function marshals task_spwan() parameters and invokes the
 *   nx_task_spawn() system call.
 *
 *   task_spawn() and posix_spawn() are NuttX OS interfaces.  In PROTECTED
 *   and KERNEL build modes, then can be reached from applications only via
 *   a system call.  Currently, the number of parameters in a system call
 *   is limited to six; these spawn function have seven parameters.  Rather
 *   than extend the maximum number of parameters across all architectures,
 *   I opted instead to marshal the seven parameters into a structure.
 *
 *
 * Input Parameters:
 *   file_actions - The address of the posix_spawn_file_actions_t to be
 *   initialized.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int task_spawn(FAR pid_t *pid, FAR const char *name, main_t entry,
               FAR const posix_spawn_file_actions_t *file_actions,
               FAR const posix_spawnattr_t *attr,
               FAR char * const argv[], FAR char * const envp[])
{
  struct spawn_syscall_parms_s parms;

  parms.pid          = pid;
  parms.name         = name;
  parms.entry        = entry;
  parms.file_actions = file_actions;
  parms.attr         = attr;
  parms.argv         = (FAR char * const *)argv;
  parms.envp         = (FAR char * const *)envp;

  return nx_task_spawn(&parms);
}

#endif /* CONFIG_LIB_SYSCALL && !CONFIG_BUILD_KERNEL */
