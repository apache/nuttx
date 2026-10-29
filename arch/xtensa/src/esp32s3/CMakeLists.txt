# ##############################################################################
# arch/xtensa/src/esp32s3/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

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

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS esp32s3_userspace.c)
endif()

if(NOT CONFIG_BUILD_FLAT)
  list(APPEND SRCS esp32s3_isolation.c)
endif()

if(CONFIG_ESP32S3_USERFAULT_ABORT)
  list(APPEND SRCS esp32s3_userfault.c)
endif()

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS esp32s3_world1_vectors.S)
endif()

if(CONFIG_BUILD_PROTECTED OR CONFIG_ARCH_ADDRENV)
  list(APPEND SRCS esp32s3_mmu.c esp32s3_pms.c esp32s3_wcl.c)
endif()

if(CONFIG_ESP32S3_PAGEFAULT)
  list(APPEND SRCS esp32s3_pagefault.c)
endif()

if(CONFIG_ARCH_ADDRENV)
  list(APPEND SRCS esp32s3_addrenv.c esp32s3_addrenv_utils.c)
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS esp32s3_pgalloc.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS esp32s3_cpuidlestack.c esp32s3_cpustart.c
       esp32s3_intercpu_interrupt.c)
endif()

if(CONFIG_ESP32S3_RNG)
  list(APPEND SRCS esp32s3_rng.c)
endif()

if(CONFIG_ESP32S3_TWAI)
  list(APPEND SRCS esp32s3_twai.c)
endif()

if(CONFIG_ESP32S3_USBSERIAL)
  list(APPEND SRCS esp32s3_usbserial.c)
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS esp32s3_tickless.c)
else()
  list(APPEND SRCS esp32s3_timerisr.c)
endif()

if(CONFIG_ESP32S3_TIMER)
  list(APPEND SRCS esp32s3_tim.c)
  if(CONFIG_TIMER)
    list(APPEND SRCS esp32s3_tim_lowerhalf.c)
  endif()
endif()

if(CONFIG_WATCHDOG)
  list(APPEND SRCS esp32s3_wdt_lowerhalf.c)
endif()

if(CONFIG_ESP32S3_ONESHOT)
  list(APPEND SRCS esp32s3_oneshot.c)
  if(CONFIG_ONESHOT)
    list(APPEND SRCS esp32s3_oneshot_lowerhalf.c)
  endif()
endif()

if(CONFIG_ESP32S3_FREERUN)
  list(APPEND SRCS esp32s3_freerun.c)
endif()

if(CONFIG_ESP32S3_DMA)
  list(APPEND SRCS esp32s3_dma.c)
endif()

if(CONFIG_ESP32S3_I2C AND CONFIG_ESPRESSIF_I2C_PERIPH_MASTER_MODE)
  list(APPEND SRCS esp32s3_i2c.c)
endif()

if(CONFIG_ESP32S3_SPI)
  if(CONFIG_ESP32S3_SPI_IO_SPI)
    list(APPEND SRCS esp32s3_spi.c)
  else()
    list(APPEND SRCS esp32s3_qspi.c)
  endif()
  if(CONFIG_SPI_SLAVE)
    list(APPEND SRCS esp32s3_spi_slave.c)
  endif()
endif()

if(CONFIG_ESP32S3_SPIFLASH AND (CONFIG_ESP32S3_PARTITION_TABLE
                                OR CONFIG_ESP32S3_SPIRAM))
  list(APPEND SRCS esp32s3_spiflash.c esp32s3_spiflash_mtd.c)
endif()

if(CONFIG_ESP32S3_PARTITION_TABLE)
  list(APPEND SRCS esp32s3_partition.c)
endif()

if(CONFIG_ESP32S3_SPIRAM)
  list(APPEND SRCS esp32s3_spiram.c esp32s3_himem.c)
  if(CONFIG_ESP32S3_SPIRAM_MODE_QUAD)
    list(APPEND SRCS esp32s3_psram_quad.c)
  endif()
  if(CONFIG_ESP32S3_SPIRAM_MODE_OCT)
    list(APPEND SRCS esp32s3_psram_octal.c)
  endif()
endif()

if(CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP)
  list(APPEND SRCS esp32s3_imm.c)
endif()

if(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
  list(APPEND SRCS esp32s3_extraheaps.c)
endif()

if(CONFIG_ESP32S3_RTC_HEAP)
  list(APPEND SRCS esp32s3_rtcheap.c)
endif()

if(CONFIG_ARCH_USE_TEXT_HEAP)
  list(APPEND SRCS esp32s3_textheap.c)
endif()

if(CONFIG_ESP32S3_TOUCH)
  list(APPEND SRCS esp32s3_touch.c)
endif()

if(CONFIG_ESP32S3_OTG_DEVICE)
  list(APPEND SRCS esp32s3_otg_device.c)
endif()

if(CONFIG_ESP32S3_LCD)
  list(APPEND SRCS esp32s3_lcd.c)
endif()

if(CONFIG_ESP32S3_CAM)
  list(APPEND SRCS esp32s3_cam.c)
endif()

if(CONFIG_ESP32S3_SDMMC)
  list(APPEND SRCS esp32s3_sdmmc.c)
endif()

if(CONFIG_ESP32S3_OPENETH)
  list(APPEND SRCS esp_openeth.c)
endif()

if(CONFIG_ESPRESSIF_WIFI)
  list(APPEND SRCS esp32s3_wifi_adapter.c)
endif()

if(CONFIG_ESPRESSIF_BLE)
  list(APPEND SRCS esp32s3_ble_adapter.c esp32s3_ble.c)
endif()

target_compile_options(
  arch
  PRIVATE -Wno-undef -Wno-unused-variable -fno-jump-tables
          -fno-tree-switch-conversion -Wno-deprecated-declarations
          -fstrict-volatile-bitfields)
target_compile_definitions(arch PRIVATE _RETARGETABLE_LOCKING)

target_sources(arch PRIVATE ${SRCS})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/espressif espressif)
