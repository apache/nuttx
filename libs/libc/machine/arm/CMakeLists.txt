############################################################################
# libs/libc/machine/arm/CMakeLists.txt
#
#   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

if (CONFIG_ARCH_ARM7TDMI)        # ARM7TDMI is ARMv4T
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARM920T)    # ARM920T is ARMv4T
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARM926EJS)  # ARM926EJS is ARMv5TEJ
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARM1136J)   # ARM1136J is ARMv6
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARM1156T2)  # ARM1156T2 is ARMv6T2
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARM1176JZ)  # ARM1176JZ is ARMv6Z
  add_subdirectory(arm)
elseif (CONFIG_ARCH_ARMV7A)     # All ARMv7-A
  add_subdirectory(armv7-a)
elseif (CONFIG_ARCH_ARMV7R)     # All ARMv7-R
  add_subdirectory(armv7-r)
elseif (CONFIG_ARCH_ARMV6M)     # All ARMv6-M
  add_subdirectory(armv6-m)
elseif (CONFIG_ARCH_ARMV7M)     # All ARMv7-M
  add_subdirectory(armv7-m)
elseif (CONFIG_ARCH_ARMV8M)     # All ARMv8-M
  add_subdirectory(armv8-m)
endif()

if (CONFIG_CXX_EXCEPTION)
  target_sources(c PRIVATE gnu_unwind_find_exidx.c)
endif()
