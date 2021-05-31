############################################################################
# libs/libc/unistd/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(SRCS
  lib_access.c lib_daemon.c lib_swab.c lib_pathconf.c lib_sysconf.c
  lib_getopt_common.c lib_getopt.c lib_getopt_long.c
  lib_getopt_longonly.c lib_getoptvars.c lib_getoptargp.c
  lib_getopterrp.c
  lib_getoptindp.c lib_getoptoptp.c
  lib_alarm.c lib_fstatvfs.c lib_statvfs.c lib_sleep.c
  lib_usleep.c lib_seteuid.c lib_setegid.c lib_geteuid.c lib_getegid.c
  lib_setreuid.c lib_setregid.c
  lib_getrusage.c lib_utimes.c
  lib_setrlimit.c lib_getrlimit.c
  lib_setpriority.c lib_getpriority.c
  lib_futimes.c
  lib_gethostname.c lib_sethostname.c)

if (NOT CONFIG_SCHED_USER_IDENTITY)
  list(APPEND SRCS lib_setuid.c lib_setgid.c lib_getuid.c lib_getgid.c)
endif()

if (NOT CONFIG_DISABLE_ENVIRON)
  list(APPEND SRCS lib_chdir.c lib_getcwd.c lib_restoredir.c)
endif()

if (CONFIG_LIBC_EXECFUNCS)
  list(APPEND SRCS lib_execl.c)
endif()

if (NOT CONFIG_DISABLE_MOUNTPOINTS)
  list(APPEND SRCS lib_truncate.c lib_posix_fallocate.c)
endif()

if (CONFIG_PIPES)
  list(APPEND SRCS lib_pipe.c lib_pipe2.c)
endif()

target_sources(c PRIVATE ${SRCS})
