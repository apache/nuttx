############################################################################
# drivers/leds/CMakeLists.txt
# Various LED-related drivers
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

set(SRCS)

if (CONFIG_USERLED)
  list(APPEND SRCS userled_upper.c)

  if (CONFIG_USERLED_LOWER)
    list(APPEND SRCS userled_lower.c)
  endif()
endif()

if (CONFIG_LEDS_APA102)
  list(APPEND SRCS apa102.c)
endif()

if (CONFIG_LEDS_MAX7219)
  list(APPEND SRCS max7219.c)
endif()

if (CONFIG_RGBLED)
  list(APPEND SRCS rgbled.c)
endif()

if (CONFIG_PCA9635PW)
  list(APPEND SRCS pca9635pw.c)
endif()

if (CONFIG_NCP5623C)
  list(APPEND SRCS ncp5623c.c)
endif()

if (CONFIG_WS2812)
  list(APPEND SRCS ws2812.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
