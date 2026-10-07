# ##############################################################################
# arch/xtensa/src/esp32s3/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/espressif espressif)

set(SRCS
    esp32s3_start.c
    esp32s3_cache.c
    esp32s3_region.c
    esp32s3_systemreset.c
    esp32s3_user.c
    esp32s3_allocateheap.c
    esp32s3_reset_reasons.c
    esp32s3_wdt.c
    esp32s3_lowputc.c
    esp32s3_serial.c
    esp32s3_rtc_gpio.c
    esp32s3_libc_stubs.c
    esp32s3_spi_timing.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS esp32s3_idle.c)
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp32s3_tickless.c)
else()
  list(APPEND SRCS esp32s3_timerisr.c)
endif()

if(CONFIG_ESPRESSIF_WIFI)
  list(APPEND SRCS esp32s3_wifi_adapter.c)
endif()

if(CONFIG_ESPRESSIF_BLE)
  list(APPEND SRCS esp32s3_ble_adapter.c esp32s3_ble.c)
endif()

target_compile_definitions(arch PRIVATE _RETARGETABLE_LOCKING ESP_PLATFORM=1)
target_sources(arch PRIVATE ${SRCS})
