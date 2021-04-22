############################################################################
# arch/arm/src/CMakeLists.txt
#
#   Copyright (C) 2007-2009, 2011-2012, 2014-2016 Gregory Nutt. All rights
#     reserved.
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

set(ARCH_SUBDIR)

if (CONFIG_ARCH_ARMV7A)         # ARMv7-A
  set(ARCH_SUBDIR armv7-a)
elseif (CONFIG_ARCH_ARMV7R)     # ARMv7-R
  set(ARCH_SUBDIR armv7-r)
elseif (CONFIG_ARCH_ARMV7M)     # ARMv7-M
  set(ARCH_SUBDIR armv7-m)
elseif (CONFIG_ARCH_ARMV8M)     # ARMv8-M
  set(ARCH_SUBDIR armv8-m)
elseif (CONFIG_ARCH_ARMV6M)     # ARMv6-M
  set(ARCH_SUBDIR armv6-m)
else()                          # ARM9, ARM7TDMI, etc.
  set(ARCH_SUBDIR arm)
endif()

add_subdirectory(${ARCH_SUBDIR})
add_subdirectory(${CONFIG_ARCH_CHIP})
add_subdirectory(common)

# include directories
target_include_directories(arch PRIVATE ${ARM_SUBDIR})
target_include_directories(arch PRIVATE ${CONFIG_ARCH_CHIP})
target_include_directories(arch PRIVATE common)
