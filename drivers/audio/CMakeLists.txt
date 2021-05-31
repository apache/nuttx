############################################################################
# drivers/audio/CMakeLists.txt
# These drivers support various Audio devices using the NuttX Audio
# interface.
#
#   Copyright (C) 2013 Ken Pettit. All rights reserved.
#   Author: Ken Pettit <pettitkd@gmail.com>
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

if (CONFIG_DRIVERS_AUDIO)
  set(SRCS)

  if (CONFIG_AUDIO_CXD56)
    list(APPEND SRCS cxd56.c)
    if (CONFIG_AUDIO_CXD56_SRC)
      list(APPEND SRCS cxd56_src.c)
    endif()
  endif()

  if (CONFIG_AUDIO_VS1053)
    list(APPEND SRCS vs1053.c)
  endif()

  if (CONFIG_AUDIO_CS43L22)
    list(APPEND SRCS cs43l22.c)

    if (CONFIG_CS43L22_REGDUMP)
      list(APPEND SRCS cs43l22_debug.c)
    elseif(CONFIG_CS43L22_CLKDEBUG)
      list(APPEND SRCS cs43l22_debug.c)
    endif()
  endif()

  if (CONFIG_AUDIO_CS4344)
    list(APPEND SRCS cs4344.c)
  endif()

  if (CONFIG_AUDIO_WM8994)
    list(APPEND SRCS wm8994.c)

    if (CONFIG_WM8994_REGDUMP)
      list(APPEND SRCS wm8994_debug.c)
    elseif (CONFIG_WM8994_CLKDEBUG)
      list(APPEND SRCS wm8994_debug.c)
    endif()
  endif()

  if (CONFIG_AUDIO_WM8904)
    list(APPEND SRCS wm8904.c)

    if (CONFIG_WM8904_REGDUMP)
      list(APPEND SRCS wm8904_debug.c)
    elseif (CONFIG_WM8904_CLKDEBUG)
      list(APPEND SRCS wm8904_debug.c)
    endif()
  endif()

  if (CONFIG_AUDIO_WM8776)
    list(APPEND SRCS wm8776.c)
  endif()

  if (CONFIG_AUDIO_NULL)
    list(APPEND SRCS audio_null.c)
  endif()

  if (CONFIG_AUDIO_TONE)
    list(APPEND SRCS tone.c)
  endif()

  if (CONFIG_AUDIO_I2S)
    list(APPEND SRCS audio_i2s.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()

