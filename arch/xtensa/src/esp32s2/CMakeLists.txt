# ##############################################################################
# arch/xtensa/src/esp32s2/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/espressif espressif)

set(SRCS
    esp32s2_start.c
    esp32s2_wdt.c
    esp32s2_allocateheap.c
    esp32s2_rtc_gpio.c
    esp32s2_region.c
    esp32s2_user.c
    esp32s2_lowputc.c
    esp32s2_systemreset.c
    esp32s2_dma.c
    esp32s2_libc_stubs.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS esp32s2_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp32s2_timerisr.c)
endif()

if(CONFIG_ESPRESSIF_WIFI)
  list(APPEND SRCS esp32s2_wifi_adapter.c)
endif()

target_compile_definitions(arch PRIVATE ESP_PLATFORM=1)
target_sources(arch PRIVATE ${SRCS})
