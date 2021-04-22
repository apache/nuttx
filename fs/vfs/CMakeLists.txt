############################################################################
# fs/vfs/Make.defs
#
#   Copyright (C) 2014-2015, 2017-2018 Gregory Nutt. All rights reserved.
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
  # Common file/socket descriptor support
  fs_close.c fs_dup.c fs_dup2.c fs_fcntl.c
  fs_epoll.c fs_fstat.c fs_fstatfs.c fs_ioctl.c
  fs_lseek.c fs_mkdir.c fs_open.c fs_poll.c  fs_read.c fs_rename.c
  fs_rmdir.c fs_statfs.c fs_stat.c fs_select.c fs_unlink.c fs_write.c

  # Support for positional file access
  fs_pread.c fs_pwrite.c
  )

# Certain interfaces are not available if there is no mountpoint support

if (NOT CONFIG_DISABLE_MOUNTPOINT)
  list(APPEND SRCS fs_fsync.c fs_truncate.c)
endif()

if (NOT "${CONFIG_PSEUDOFS_SOFTLINKS}" STREQUAL "0")
  list(APPEND SRCS fs_symlink.c fs_readlink.c)
endif()

# Stream support

if (CONFIG_FILE_STREAM)
  list(APPEND SRCS fs_fdopen.c)
endif()

# Support for sendfile()

if (CONFIG_NET_SENDFILE)
  list(APPEND SRCS fs_sendfile.c)
endif()

# Support for eventfd

if (CONFIG_EVENT_FD)
  list(APPEND SRCS fs_eventfd.c)
endif()

target_sources(fs PRIVATE ${SRCS})
