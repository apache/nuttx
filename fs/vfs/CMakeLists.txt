# ##############################################################################
# fs/vfs/CMakeLists.txt
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
    fs_chstat.c
    fs_close.c
    fs_dup.c
    fs_dup2.c
    fs_fcntl.c
    fs_epoll.c
    fs_fchstat.c
    fs_fstat.c
    fs_fstatfs.c
    fs_ioctl.c
    fs_lseek.c
    fs_mkdir.c
    fs_open.c
    fs_poll.c
    fs_pread.c
    fs_pwrite.c
    fs_read.c
    fs_rename.c
    fs_rmdir.c
    fs_select.c
    fs_stat.c
    fs_sendfile.c
    fs_statfs.c
    fs_uio.c
    fs_unlink.c
    fs_write.c
    fs_dir.c
    fs_fsync.c
    fs_syncfs.c
    fs_truncate.c)

# File lock support

if(NOT "${CONFIG_FS_LOCK_BUCKET_SIZE}" STREQUAL "0")
  list(APPEND SRCS fs_lock.c)
endif()

# Certain interfaces are not available if there is no mountpoint support

if(NOT "${CONFIG_PSEUDOFS_SOFTLINKS}" STREQUAL "0")
  list(APPEND SRCS fs_link.c fs_symlink.c fs_readlink.c)
endif()

# Pseudofile support

if(CONFIG_PSEUDOFS_FILE)
  list(APPEND SRCS fs_pseudofile.c)
endif()

# Support for eventfd

if(CONFIG_EVENT_FD)
  list(APPEND SRCS fs_eventfd.c)
endif()

# Support for timerfd

if(CONFIG_TIMER_FD)
  list(APPEND SRCS fs_timerfd.c)
endif()

# Support for signalfd

if(CONFIG_SIGNAL_FD)
  list(APPEND SRCS fs_signalfd.c)
endif()

target_sources(fs PRIVATE ${SRCS})
