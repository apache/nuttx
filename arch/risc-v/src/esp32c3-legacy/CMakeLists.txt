# ##############################################################################
# arch/risc-v/src/esp32c3-legacy/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/espressif espressif)

set(SRCS
    esp32c3_head.S
    esp32c3_vectors.S
    esp32c3_allocateheap.c
    esp32c3_start.c
    esp32c3_wdt.c
    esp32c3_idle.c
    esp32c3_irq.c
    esp32c3_libc_stubs.c
    esp32c3_clockconfig.c
    esp32c3_gpio.c
    esp32c3_rtc_gpio.c
    esp32c3_lowputc.c
    esp32c3_serial.c
    esp32c3_systemreset.c
    esp32c3_resetcause.c
    esp32c3_uid.c
    esp32c3_perf.c
    esp32c3_rtc.c)

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp32c3_tickless.c)
else()
  list(APPEND SRCS esp32c3_timerisr.c)
endif()

if(CONFIG_ESPRESSIF_WIFI)
  list(APPEND SRCS esp32c3_wifi_adapter.c)
endif()

if(CONFIG_ESPRESSIF_BLE)
  list(APPEND SRCS esp32c3_ble_adapter.c esp32c3_ble.c)
endif()

target_compile_definitions(arch PRIVATE _RETARGETABLE_LOCKING)
target_sources(arch PRIVATE ${SRCS})
