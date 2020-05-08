/****************************************************************************
 * syscall/syscall_funclookup.c
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
#include <syscall.h>

/* The content of this file is only meaningful during the kernel phase of
 * a kernel build.
 */

#if defined(CONFIG_LIB_SYSCALL) && defined(__KERNEL__)

#include <nuttx/arch.h>
#include <nuttx/module.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/spawn.h>
#include <nuttx/tls.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>

#include <sys/boardctl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/random.h>
#include <sys/select.h>
#include <sys/sendfile.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/time.h>
#include <sys/utsname.h>
#include <sys/wait.h>

#include <aio.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
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
#  define SYSCALL_LOOKUP1(f,n) (uintptr_t)f
#  define SYSCALL_LOOKUP(f,n)  , (uintptr_t)f
#  include <sys/syscall_lookup.h>
#  undef SYSCALL_LOOKUP1
#  undef SYSCALL_LOOKUP
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_LIB_SYSCALL && __KERNEL__ */
