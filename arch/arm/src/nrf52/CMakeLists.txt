# ##############################################################################
# arch/arm/src/nrf52/CMakeLists.txt
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
    nrf52_start.c
    nrf52_clockconfig.c
    nrf52_irq.c
    nrf52_utils.c
    nrf52_allocateheap.c
    nrf52_lowputc.c
    nrf52_gpio.c
    nrf52_uid.c)

if(CONFIG_NRF52_SYSTIMER_SYSTICK)
  list(APPEND SRCS nrf52_systick.c)
endif()

if(CONFIG_NRF52_SYSTIMER_RTC)
  list(APPEND SRCS nrf52_tickless_rtc.c)
endif()

if(CONFIG_ALLOW_BSD_COMPONENTS)
  list(APPEND SRCS nrf52_nvmc.c)

  if(CONFIG_NRF52_PROGMEM)
    list(APPEND SRCS nrf52_flash.c)
  endif()
endif()

if(CONFIG_ARCH_CHIP_NRF52832)
  list(APPEND SRCS nrf52832_errdata.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS nrf52_userspace.c nrf52_mpuinit.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nrf52_idle.c)
endif()

if(CONFIG_NRF52_GPIOTE)
  list(APPEND SRCS nrf52_gpiote.c)
endif()

if(CONFIG_NRF52_UART)
  list(APPEND SRCS nrf52_serial.c)
endif()

if(CONFIG_NRF52_WDT)
  list(APPEND SRCS nrf52_wdt.c)

  if(CONFIG_WATCHDOG)
    list(APPEND SRCS nrf52_wdt_lowerhalf.c)
  endif()
endif()

if(CONFIG_NRF52_RNG)
  list(APPEND SRCS nrf52_rng.c)
endif()

if(CONFIG_NRF52_SPI_MASTER)
  list(APPEND SRCS nrf52_spi.c)
endif()

if(CONFIG_NRF52_I2C_MASTER)
  list(APPEND SRCS nrf52_i2c.c)
endif()

if(CONFIG_I2C_BITBANG)
  list(APPEND SRCS nrf52_i2c_bitbang.c)
endif()

if(CONFIG_NRF52_PPI)
  list(APPEND SRCS nrf52_ppi.c)
endif()

if(CONFIG_NRF52_RADIO)
  list(APPEND SRCS nrf52_radio.c)

  if(CONFIG_NRF52_RADIO_IEEE802154)
    list(APPEND SRCS nrf52_ieee802154_radio.c)
    list(APPEND SRCS nrf52_ieee802154_tim.c)
    list(APPEND SRCS nrf52_ieee802154.c)

    if(CONFIG_NRF52_RADIO_IEEE802154_SUPERFRAME)
      list(APPEND SRCS nrf52_ieee802154_rtc.c)
    endif()

    if(CONFIG_NRF52_RADIO_IEEE802154_TRACE)
      list(APPEND SRCS nrf52_ieee802154_trace.c)
    endif()
  endif()
endif()

if(CONFIG_NRF52_TIMER)
  list(APPEND SRCS nrf52_tim.c)

  if(CONFIG_TIMER)
    list(APPEND SRCS nrf52_tim_lowerhalf.c)
  endif()
endif()

if(CONFIG_NRF52_RTC)
  list(APPEND SRCS nrf52_rtc.c)
endif()

if(CONFIG_NRF52_PWM)
  list(APPEND SRCS nrf52_pwm.c)
endif()

if(CONFIG_NRF52_SAADC)
  list(APPEND SRCS nrf52_adc.c)
endif()

if(CONFIG_PM)
  list(APPEND SRCS nrf52_pminitialize.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS nrf52_usbd.c)
endif()

if(CONFIG_NRF52_QSPI)
  list(APPEND SRCS nrf52_qspi.c)
endif()

if(CONFIG_NRF52_SOFTDEVICE_CONTROLLER)

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

  list(APPEND SRCS nrf52_sdc.c)

  add_compile_options(-DNRF52_SERIES)

  if(CONFIG_ARCH_FPU)
    set(NRFXLIB_LIB_VARIANT hard-float)
  else()
    set(NRFXLIB_LIB_VARIANT soft-float)
  endif()

  if(CONFIG_NRF52_SDC_PERIPHERAL)
    set(SOFTDEVICE_LIB_VARIANT libsoftdevice_controller_peripheral.a)
  elseif(CONFIG_NRF52_SDC_CENTRAL)
    set(SOFTDEVICE_LIB_VARIANT libsoftdevice_controller_central.a)
  elseif(CONFIG_NRF52_SDC_MULTIROLE)
    set(SOFTDEVICE_LIB_VARIANT libsoftdevice_controller_multirole.a)
  endif()

  target_link_libraries(
    arch ${NRFXLIB_DIR}/mpsl/lib/cortex-m4/${NRFXLIB_LIB_VARIANT}/libmpsl.a)
  target_link_libraries(
    arch
    ${NRFXLIB_DIR}/softdevice_controller/lib/cortex-m4/${NRFXLIB_LIB_VARIANT}/${SOFTDEVICE_LIB_VARIANT}
  )

endif()

target_sources(arch PRIVATE ${SRCS})
