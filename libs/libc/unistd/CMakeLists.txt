# ##############################################################################
# libs/libc/unistd/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS
    lib_access.c
    lib_daemon.c
    lib_swab.c
    lib_pathconf.c
    lib_sysconf.c
    lib_getcwd.c
    lib_getentropy.c
    lib_getopt_common.c
    lib_getopt.c
    lib_getopt_long.c
    lib_getopt_longonly.c
    lib_getoptvars.c
    lib_getoptargp.c
    lib_getopterrp.c
    lib_getoptindp.c
    lib_getoptoptp.c
    lib_times.c
    lib_alarm.c
    lib_fstatvfs.c
    lib_statvfs.c
    lib_sleep.c
    lib_nice.c
    lib_setreuid.c
    lib_setregid.c
    lib_getrusage.c
    lib_utime.c
    lib_utimes.c
    lib_setrlimit.c
    lib_getrlimit.c
    lib_setpriority.c
    lib_getpriority.c
    lib_futimes.c
    lib_lutimes.c
    lib_gethostname.c
    lib_sethostname.c
    lib_fchownat.c
    lib_linkat.c
    lib_readlinkat.c
    lib_symlinkat.c
    lib_unlinkat.c
    lib_usleep.c
    lib_getpgrp.c
    lib_getpgid.c
    lib_lockf.c
    lib_flock.c
    lib_getpass.c
    lib_chdir.c
    lib_fchdir.c)

if(NOT CONFIG_SCHED_USER_IDENTITY)
  list(
    APPEND
    SRCS
    lib_setuid.c
    lib_setgid.c
    lib_getuid.c
    lib_getgid.c
    lib_seteuid.c
    lib_setegid.c
    lib_geteuid.c
    lib_getegid.c)
endif()

if(NOT CONFIG_DISABLE_ENVIRON)
  list(APPEND SRCS lib_restoredir.c)
endif()

if(CONFIG_LIBC_EXECFUNCS)
  list(APPEND SRCS lib_execl.c lib_execle.c lib_execv.c)
endif()

if(NOT CONFIG_DISABLE_MOUNTPOINTS)
  list(APPEND SRCS lib_truncate.c lib_posix_fallocate.c)
endif()

if(CONFIG_ARCH_HAVE_FORK)
  list(APPEND SRCS lib_fork.c)
endif()

if(CONFIG_CRYPTO)
  list(APPEND SRCS lib_crypt.c lib_crypt_r.c)
endif()

target_sources(c PRIVATE ${SRCS})
