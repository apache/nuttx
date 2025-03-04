# ##############################################################################
# arch/arm/src/nrf53/CMakeLists.txt
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
    nrf53_start.c
    nrf53_clockconfig.c
    nrf53_irq.c
    nrf53_utils.c
    nrf53_allocateheap.c
    nrf53_lowputc.c
    nrf53_gpio.c
    nrf53_uid.c
    nrf53_spu.c)

if(CONFIG_NRF53_SYSTIMER_SYSTICK)
  list(APPEND SRCS nrf53_systick.c)
endif()

if(CONFIG_NRF53_SYSTIMER_RTC)
  list(APPEND SRCS nrf53_tickless_rtc.c)
endif()

if(CONFIG_NRF53_APPCORE)
  list(APPEND SRCS nrf53_oscconfig.c nrf53_cpunet.c)
endif()

if(CONFIG_NRF53_PROGMEM)
  list(APPEND SRCS nrf53_flash.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nrf53_idle.c)
endif()

if(CONFIG_NRF53_GPIOTE)
  list(APPEND SRCS nrf53_gpiote.c)
endif()

if(CONFIG_NRF53_UART)
  list(APPEND SRCS nrf53_serial.c)
endif()

if(CONFIG_NRF53_IPC)
  list(APPEND SRCS nrf53_ipc.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS nrf53_rptun.c)
endif()

if(CONFIG_NRF53_SPI_MASTER)
  list(APPEND SRCS nrf53_spi.c)
endif()

if(CONFIG_NRF53_I2C_MASTER)
  list(APPEND SRCS nrf53_i2c.c)
endif()

if(CONFIG_NRF53_TIMER)
  list(APPEND SRCS nrf53_tim.c)

  if(CONFIG_TIMER)
    list(APPEND SRCS nrf53_tim_lowerhalf.c)
  endif()
endif()

if(CONFIG_NRF53_RTC)
  list(APPEND SRCS nrf53_rtc.c)
endif()

if(CONFIG_NRF53_PWM)
  list(APPEND SRCS nrf53_pwm.c)
endif()

if(CONFIG_NRF53_SAADC)
  list(APPEND SRCS nrf53_adc.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS nrf53_pminitialize.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS nrf53_usbd.c)
endif()

if(CONFIG_NRF53_QSPI)
  list(APPEND SRCS nrf53_qspi.c)
endif()

if(CONFIG_NRF53_SOFTDEVICE_CONTROLLER)

  set(NRFXLIB_VER "2.3.0")
  set(NRFXLIB_URL "https://github.com/nrfconnect/sdk-nrfxlib/archive")

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/sdk-nrfxlib)
    FetchContent_Declare(
      sdk-nrfxlib
      DOWNLOAD_NAME "sdk-nrfxlib-v${NRFXLIB_VER}.tar.gz"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "${NRFXLIB_URL}/v${NRFXLIB_VER}.tar.gz"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/sdk-nrfxlib
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/arch/sdk-nrfxlib
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(sdk-nrfxlib)

    if(NOT sdk-nrfxlib_POPULATED)
      FetchContent_Populate(sdk-nrfxlib)
    endif()
  endif()

  set(NRFXLIB_DIR "${NUTTX_CHIP_ABS_DIR}/sdk-nrfxlib")

  target_include_directories(arch PRIVATE ${NUTTX_CHIP_ABS_DIR}/sdc)
  target_include_directories(arch PRIVATE ${NRFXLIB_DIR}/mpsl/include)
  target_include_directories(
    arch PRIVATE ${NRFXLIB_DIR}/softdevice_controller/include)

  list(APPEND SRCS nrf53_sdc.c)

  add_compile_options(-DNRF53_SERIES)

  if(CONFIG_ARCH_FPU)
    set(NRFXLIB_LIB_VARIANT hard-float)
  else()
    set(NRFXLIB_LIB_VARIANT soft-float)
  endif()

  set(SOFTDEVICE_LIB_VARIANT libsoftdevice_controller_multirole.a)

  target_link_libraries(
    arch
    ${NRFXLIB_DIR}/mpsl/lib/cortex-m33+nodsp/${NRFXLIB_LIB_VARIANT}/libmpsl.a)
  target_link_libraries(
    arch
    ${NRFXLIB_DIR}/softdevice_controller/lib/cortex-m33+nodsp/${NRFXLIB_LIB_VARIANT}/${SOFTDEVICE_LIB_VARIANT}
  )

endif()

target_sources(arch PRIVATE ${SRCS})
