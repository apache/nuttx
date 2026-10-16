# ##############################################################################
# arch/xtensa/src/esp32/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/espressif espressif)

set(SRCS
    esp32_start.c
    esp32_wdt.c
    esp32_allocateheap.c
    esp32_systemreset.c
    esp32_resetcause.c
    esp32_region.c
    esp32_rtc_gpio.c
    esp32_user.c
    esp32_libc_stubs.c
    esp32_dma.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS esp32_idle.c)
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp32_tickless.c)
else()
  list(APPEND SRCS esp32_timerisr.c)
endif()

if(CONFIG_ESPRESSIF_WIFI)
  list(APPEND SRCS esp32_wifi_adapter.c)
endif()

if(CONFIG_ESPRESSIF_BLE)
  list(APPEND SRCS esp32_ble_adapter.c esp32_ble.c)
endif()

if(CONFIG_XTENSA_HAVE_WINDOW_EXCEPTION_HOOKS)
  list(APPEND SRCS esp32_window_hooks.S)
endif()

target_compile_definitions(arch PRIVATE ESP_PLATFORM=1)
target_sources(arch PRIVATE ${SRCS})
