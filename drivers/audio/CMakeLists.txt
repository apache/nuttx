# ##############################################################################
# drivers/audio/CMakeLists.txt
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
#
# ##############################################################################

if(CONFIG_DRIVERS_AUDIO)
  set(SRCS)

  if(CONFIG_AUDIO_CXD56)
    list(APPEND SRCS cxd56.c)
    if(CONFIG_AUDIO_CXD56_SRC)
      list(APPEND SRCS cxd56_src.c)
    endif()
  endif()

  if(CONFIG_AUDIO_VS1053)
    list(APPEND SRCS vs1053.c)
  endif()

  if(CONFIG_AUDIO_CS43L22)
    list(APPEND SRCS cs43l22.c)

    if(CONFIG_CS43L22_REGDUMP)
      list(APPEND SRCS cs43l22_debug.c)
    elseif(CONFIG_CS43L22_CLKDEBUG)
      list(APPEND SRCS cs43l22_debug.c)
    endif()
  endif()

  if(CONFIG_AUDIO_CS4344)
    list(APPEND SRCS cs4344.c)
  endif()

  if(CONFIG_AUDIO_ES8311)
    list(APPEND SRCS es8311.c)
    if(CONFIG_ES8311_REGDUMP)
      list(APPEND SRCS es8311_debug.c)
    endif()
  endif()

  if(CONFIG_AUDIO_ES8388)
    list(APPEND SRCS es8388.c)
    if(CONFIG_ES8388_REGDUMP)
      list(APPEND SRCS es8388_debug.c)
    endif()
  endif()

  if(CONFIG_AUDIO_WM8994)
    list(APPEND SRCS wm8994.c)

    if(CONFIG_WM8994_REGDUMP)
      list(APPEND SRCS wm8994_debug.c)
    elseif(CONFIG_WM8994_CLKDEBUG)
      list(APPEND SRCS wm8994_debug.c)
    endif()
  endif()

  if(CONFIG_AUDIO_WM8904)
    list(APPEND SRCS wm8904.c)

    if(CONFIG_WM8904_REGDUMP)
      list(APPEND SRCS wm8904_debug.c)
    elseif(CONFIG_WM8904_CLKDEBUG)
      list(APPEND SRCS wm8904_debug.c)
    endif()
  endif()

  if(CONFIG_AUDIO_WM8776)
    list(APPEND SRCS wm8776.c)
  endif()

  if(CONFIG_AUDIO_NULL)
    list(APPEND SRCS audio_null.c)
  endif()

  if(CONFIG_AUDIO_TONE)
    list(APPEND SRCS tone.c)
  endif()

  if(CONFIG_AUDIO_I2S)
    list(APPEND SRCS audio_i2s.c)
  endif()

  if(CONFIG_AUDIO_DMA)
    list(APPEND SRCS audio_dma.c)
  endif()
  target_sources(drivers PRIVATE ${SRCS})
endif()
