# ##############################################################################
# arch/arm/src/nrf54l/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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

set(SRCS
    nrf54l_start.c
    nrf54l_clockconfig.c
    nrf54l_oscconfig.c
    nrf54l_irq.c
    nrf54l_allocateheap.c
    nrf54l_gpio.c
    nrf54l_lowputc.c
    nrf54l_uid.c)

if(CONFIG_NRF54L_PROGMEM)
  list(APPEND SRCS nrf54l_flash.c)
endif()

if(CONFIG_NRF54L_SYSTIMER_SYSTICK)
  list(APPEND SRCS nrf54l_systick.c)
endif()
if(CONFIG_NRF54L_GRTC)
  list(APPEND SRCS nrf54l_grtc.c)
endif()
if(CONFIG_NRF54L_SYSTIMER_GRTC)
  list(APPEND SRCS nrf54l_tickless_grtc.c)
endif()
if(CONFIG_NRF54L_UART)
  list(APPEND SRCS nrf54l_serial.c)
endif()
if(CONFIG_NRF54L_SAADC)
  list(APPEND SRCS nrf54l_adc.c)
endif()
if(CONFIG_NRF54L_GPIOTE)
  list(APPEND SRCS nrf54l_gpiote.c)
endif()
if(CONFIG_NRF54L_PWM)
  list(APPEND SRCS nrf54l_pwm.c)
endif()
if(CONFIG_NRF54L_TIMER)
  list(APPEND SRCS nrf54l_tim.c)
  if(CONFIG_TIMER)
    list(APPEND SRCS nrf54l_tim_lowerhalf.c)
  endif()
endif()
if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nrf54l_idle.c)
endif()
if(CONFIG_PM)
  list(APPEND SRCS nrf54l_pminitialize.c)
endif()

if(CONFIG_NRF54L_SOFTDEVICE_CONTROLLER)
  set(NRFXLIB_VER "3.4.1")
  set(NRFXLIB_DIR "${NUTTX_CHIP_ABS_DIR}/sdk-nrfxlib")

  if(NOT EXISTS ${NRFXLIB_DIR})
    FetchContent_Declare(
      sdk-nrfxlib
      URL "https://github.com/nrfconnect/sdk-nrfxlib/archive/v${NRFXLIB_VER}.tar.gz"
          SOURCE_DIR
          ${NRFXLIB_DIR}
          BINARY_DIR
          ${NUTTX_BINARY_DIR}/arch/sdk-nrfxlib
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 120)
    FetchContent_GetProperties(sdk-nrfxlib)
    if(NOT sdk-nrfxlib_POPULATED)
      FetchContent_Populate(sdk-nrfxlib)
    endif()
  endif()

  list(APPEND SRCS nrf54l_sdc.c nrf54l_sdc_hci.c)
  target_include_directories(
    arch PRIVATE ${NUTTX_CHIP_ABS_DIR}/sdc ${NRFXLIB_DIR}/mpsl/include
                 ${NRFXLIB_DIR}/softdevice_controller/include)

  if(CONFIG_ARCH_CHIP_NRF54L15)
    target_compile_definitions(arch PRIVATE NRF54L15_XXAA)
    set(NRFXLIB_SOC nrf54l)
  else()
    set(NRFXLIB_SOC nrf54lm)
    if(CONFIG_ARCH_CHIP_NRF54LM20A)
      target_compile_definitions(arch PRIVATE NRF54LM20A_XXAA)
    else()
      target_compile_definitions(arch PRIVATE NRF54LM20B_XXAA)
    endif()
  endif()

  if(CONFIG_ARCH_FPU)
    set(NRFXLIB_ABI hard-float)
  else()
    set(NRFXLIB_ABI soft-float)
  endif()

  target_link_libraries(
    arch
    ${NRFXLIB_DIR}/softdevice_controller/lib/${NRFXLIB_SOC}/${NRFXLIB_ABI}/libsoftdevice_controller_multirole.a
    ${NRFXLIB_DIR}/mpsl/lib/${NRFXLIB_SOC}/${NRFXLIB_ABI}/libmpsl.a
    ${NRFXLIB_DIR}/mpsl/fem/common/lib/${NRFXLIB_SOC}/${NRFXLIB_ABI}/libmpsl_fem_common.a
  )
endif()

target_sources(arch PRIVATE ${SRCS})
