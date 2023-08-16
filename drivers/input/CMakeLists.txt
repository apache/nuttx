# ##############################################################################
# drivers/input/CMakeLists.txt
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

if(CONFIG_INPUT)
  set(SRCS)

  # Include the selected touchscreen drivers

  if(CONFIG_INPUT_TOUCHSCREEN)
    list(APPEND SRCS touchscreen_upper.c)
  endif()

  if(CONFIG_INPUT_MOUSE)
    list(APPEND SRCS mouse_upper.c)
  endif()

  if(CONFIG_INPUT_UINPUT)
    list(APPEND SRCS uinput.c)
  endif()

  if(CONFIG_INPUT_TSC2007)
    list(APPEND SRCS tsc2007.c)
  endif()

  if(CONFIG_INPUT_FT5X06)
    list(APPEND SRCS ft5x06.c)
  endif()

  if(CONFIG_INPUT_ADS7843E)
    list(APPEND SRCS ads7843e.c)
  endif()

  if(CONFIG_INPUT_MAX11802)
    list(APPEND SRCS max11802.c)
  endif()

  if(CONFIG_INPUT_MXT)
    list(APPEND SRCS mxt.c)
  endif()

  if(CONFIG_INPUT_STMPE811)
    list(APPEND SRCS stmpe811_base.c)

    if(NOT CONFIG_INPUT_STMPE811_TSC_DISABLE)
      list(APPEND SRCS stmpe811_tsc.c)
    endif()
    if(NOT CONFIG_INPUT_STMPE811_GPIO_DISABLE)
      list(APPEND SRCS stmpe811_gpio.c)
    endif()
    if(NOT CONFIG_INPUT_STMPE811_ADC_DISABLE)
      list(APPEND SRCS stmpe811_adc.c)
    endif()
    if(NOT CONFIG_INPUT_STMPE811_TEMP_DISABLE)
      list(APPEND SRCS stmpe811_temp.c)
    endif()
  endif()

  if(CONFIG_INPUT_CYPRESS_MBR3108)
    list(APPEND SRCS cypress_mbr3108.c)
  endif()

  if(CONFIG_INPUT_GT9XX)
    list(APPEND SRCS gt9xx.c)
  endif()

  if(CONFIG_INPUT_BUTTONS)
    list(APPEND SRCS button_upper.c)

    if(CONFIG_INPUT_BUTTONS_LOWER)
      list(APPEND SRCS button_lower.c)
    endif()
  endif()

  if(CONFIG_INPUT_KEYBOARD)
    list(APPEND SRCS keyboard_upper.c)
  endif()

  if(CONFIG_INPUT_DJOYSTICK)
    list(APPEND SRCS djoystick.c)
  endif()

  if(CONFIG_INPUT_AJOYSTICK)
    list(APPEND SRCS ajoystick.c)
  endif()

  if(CONFIG_INPUT_NUNCHUCK)
    list(APPEND SRCS nunchuck.c)
  endif()

  if(CONFIG_INPUT_SPQ10KBD)
    list(APPEND SRCS spq10kbd.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
