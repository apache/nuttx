############################################################################
# fs/driver/Make.defs
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

set(SRCS fs_registerdriver.c fs_unregisterdriver.c)

# Don't built-in block driver support if there are no mountpoints

if (NOT CONFIG_DISABLE_MOUNTPOINT)
  list(APPEND SRCS
    fs_registerblockdriver.c fs_unregisterblockdriver.c
    fs_findblockdriver.c fs_openblockdriver.c fs_closeblockdriver.c
    fs_blockpartition.c fs_findmtddriver.c)

  if (CONFIG_MTD)
    list(APPEND SRCS
      fs_registermtddriver.c fs_unregistermtddriver.c
      fs_mtdproxy.c)

    if (CONFIG_MTD_PARTITION)
      list(APPEND SRCS fs_mtdpartition.c)
    endif()
  endif()

  if (CONFIG_BCH)
    if (NOT CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
      list(APPEND SRCS fs_blockproxy.c)
    endif()
  endif()
endif()

target_sources(fs PRIVATE ${SRCS})
