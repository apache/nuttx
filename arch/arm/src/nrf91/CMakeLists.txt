# ##############################################################################
# arch/arm/src/nrf91/CMakeLists.txt
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
    nrf91_start.c
    nrf91_clockconfig.c
    nrf91_irq.c
    nrf91_utils.c
    nrf91_allocateheap.c
    nrf91_lowputc.c
    nrf91_gpio.c
    nrf91_uid.c
    nrf91_spu.c
    nrf91_errata.c)

if(CONFIG_NRF91_SYSTIMER_SYSTICK)
  list(APPEND SRCS nrf91_systick.c)
endif()

if(CONFIG_NRF91_SYSTIMER_RTC)
  list(APPEND SRCS nrf91_tickless_rtc.c)
endif()

if(CONFIG_NRF91_PROGMEM)
  list(APPEND SRCS nrf91_flash.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nrf91_idle.c)
endif()

if(CONFIG_NRF91_GPIOTE)
  list(APPEND SRCS nrf91_gpiote.c)
endif()

if(CONFIG_NRF91_UART)
  list(APPEND SRCS nrf91_serial.c)
endif()

if(CONFIG_NRF91_SPI_MASTER)
  list(APPEND SRCS nrf91_spi.c)
endif()

if(CONFIG_NRF91_I2C_MASTER)
  list(APPEND SRCS nrf91_i2c.c)
endif()

if(CONFIG_NRF91_TIMER)
  list(APPEND SRCS nrf91_tim.c)

  if(CONFIG_TIMER)
    list(APPEND SRCS nrf91_tim_lowerhalf.c)
  endif()
endif()

if(CONFIG_NRF91_RTC)
  list(APPEND SRCS nrf91_rtc.c)
endif()

if(CONFIG_NRF91_PWM)
  list(APPEND SRCS nrf91_pwm.c)
endif()

if(CONFIG_NRF91_SAADC)
  list(APPEND SRCS nrf91_adc.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS nrf91_pminitialize.c)
endif()

if(CONFIG_NRF91_MODEM)

  set(NRFXLIB_VER "2.4.0")
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

  list(APPEND SRCS nrf91_modem.c nrf91_modem_os.c nrf91_nrfx_ipc.c
       nrf91_modem_sock.c)

  if(CONFIG_NRF91_MODEM_AT)
    list(APPEND SRCS nrf91_modem_at.c)
  endif()

  if(CONFIG_NRF91_MODEM_GNSS)
    list(APPEND SRCS nrf91_modem_gnss.c)
  endif()

  set(NRFXLIB_DIR "${NUTTX_CHIP_ABS_DIR}/sdk-nrfxlib")

  if(CONFIG_ARCH_FPU)
    set(NRFXLIB_LIB_VARIANT hard-float)
  else()
    set(NRFXLIB_LIB_VARIANT soft-float)
  endif()

  if(CONFIG_NRF91_MODEM_LOG)
    set(MODEM_LIB_VARIANT libmodem_log.a)
  else()
    set(MODEM_LIB_VARIANT libmodem.a)
  endif()

  nuttx_library_import(
    modem
    ${NRFXLIB_DIR}/nrf_modem/lib/cortex-m33/${NRFXLIB_LIB_VARIANT}/${MODEM_LIB_VARIANT}
  )
  target_include_directories(arch PRIVATE ${NRFXLIB_DIR}/nrf_modem/include)

  target_link_libraries(arch PRIVATE modem)
  target_link_libraries(modem INTERFACE arch c net)
endif()

target_sources(arch PRIVATE ${SRCS})
