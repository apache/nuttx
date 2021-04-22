############################################################################
# drivers/input/CMakeLists.txt
#
#   Copyright (C) 2011-2012, 2015 Gregory Nutt. All rights reserved.
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

if (CONFIG_INPUT)
  set(SRCS)

  # Include the selected touchscreen drivers

  if (CONFIG_INPUT_TSC2007)
    list(APPEND SRCS tsc2007.c)
  endif()

  if (CONFIG_INPUT_FT5X06)
    list(APPEND SRCS ft5x06.c)
  endif()

  if (CONFIG_INPUT_ADS7843E)
    list(APPEND SRCS ads7843e.c)
  endif()

  if (CONFIG_INPUT_MAX11802)
    list(APPEND SRCS max11802.c)
  endif()

  if (CONFIG_INPUT_MXT)
    list(APPEND SRCS mxt.c)
  endif()

  if (CONFIG_INPUT_STMPE811)
    list(APPEND SRCS stmpe811_base.c)

    if (NOT CONFIG_INPUT_STMPE811_TSC_DISABLE)
      list(APPEND SRCS stmpe811_tsc.c)
    endif()
    if (NOT CONFIG_INPUT_STMPE811_GPIO_DISABLE)
      list(APPEND SRCS stmpe811_gpio.c)
    endif()
    if (NOT CONFIG_INPUT_STMPE811_ADC_DISABLE)
      list(APPEND SRCS stmpe811_adc.c)
    endif()
    if (NOT CONFIG_INPUT_STMPE811_TEMP_DISABLE)
      list(APPEND SRCS stmpe811_temp.c)
    endif()
  endif()

  if (CONFIG_INPUT_CYPRESS_MBR3108)
    list(APPEND SRCS cypress_mbr3108.c)
  endif()

  if (CONFIG_INPUT_BUTTONS)
    list(APPEND SRCS button_upper.c)

    if (CONFIG_INPUT_BUTTONS_LOWER)
      list(APPEND SRCS button_lower.c)
    endif()
  endif()

  if (CONFIG_INPUT_DJOYSTICK)
    list(APPEND SRCS djoystick.c)
  endif()

  if (CONFIG_INPUT_AJOYSTICK)
    list(APPEND SRCS ajoystick.c)
  endif()

  if (CONFIG_INPUT_NUNCHUCK)
    list(APPEND SRCS nunchuck.c)
  endif()

  if (CONFIG_INPUT_SPQ10KBD)
    list(APPEND SRCS spq10kbd.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
