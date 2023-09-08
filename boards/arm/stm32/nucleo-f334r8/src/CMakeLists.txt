# ##############################################################################
# boards/arm/stm32/nucleo-f334r8/src/CMakeLists.txt
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

set(SRCS stm32_boot.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_DAC)
  list(APPEND SRCS stm32_dac.c)
endif()

if(CONFIG_STM32_HRTIM)
  list(APPEND SRCS stm32_hrtim.c)
endif()

if(CONFIG_COMP)
  list(APPEND SRCS stm32_comp.c)
endif()

if(CONFIG_OPAMP)
  list(APPEND SRCS stm32_opamp.c)
endif()

if(CONFIG_NUCLEOF334R8_HIGHPRI)
  list(APPEND SRCS stm32_highpri.c)
endif()

if(CONFIG_NUCLEOF334R8_SPWM)
  list(APPEND SRCS stm32_spwm.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
