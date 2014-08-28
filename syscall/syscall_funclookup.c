/****************************************************************************
 * syscall/syscall_funclookup.c
 *
 *   Copyright (C) 2011-2014 Gregory Nutt. All rights reserved.
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
#include <syscall.h>

/* The content of this file is only meaningful during the kernel phase of
 * a kernel build.
 */

#if defined(CONFIG_LIB_SYSCALL) && defined(__KERNEL__)

#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/mount.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <poll.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <mqueue.h>
#include <spawn.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>

/* clock_systimer is a special case:  In the kernel build, proxying for
 * clock_systimer() must be handled specially.  In the kernel phase of
 * the build, clock_systimer() is macro that simply accesses a global
 * variable.  In the user phase of the kernel build, clock_systimer()
 * is a proxy function.
 *
 * In order to fill out the table g_funclookup[], this function will stand
 * in during the kernel phase of the build so that clock_systemer() will
 * have an address that can be included in the g_funclookup[] table.
 */

uint32_t syscall_clock_systimer(void);

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Function lookup tables.  This table is indexed by the system call numbers
 * defined above.  Given the system call number, this table provides the
 * address of the corresponding system function.
 *
 * This table is only available during the kernel phase of a kernel build.
 */

const uintptr_t g_funclookup[SYS_nsyscalls] =
{
#  undef SYSCALL_LOOKUP1
#  define SYSCALL_LOOKUP1(f,n,p) (uintptr_t)f
#  undef SYSCALL_LOOKUP
#  define SYSCALL_LOOKUP(f,n,p)  , (uintptr_t)f
#  include "syscall_lookup.h"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_LIB_SYSCALL && __KERNEL__ */
