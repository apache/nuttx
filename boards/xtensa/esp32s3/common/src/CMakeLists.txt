# ##############################################################################
# boards/xtensa/esp32s3/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ESP32S3_TIMER)
    list(APPEND SRCS esp32s3_board_tim.c)
  endif()
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32s3_board_wdt.c)
  endif()
  if(CONFIG_ESP32S3_SPIFLASH)
    list(APPEND SRCS esp32s3_board_spiflash.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32s3_board_spidev.c)
  endif()
  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp32s3_board_spislavedev.c)
  endif()
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp32s3_board_wlan.c)
  endif()
  if(CONFIG_I2C_DRIVER)
    list(APPEND SRCS esp32s3_board_i2c.c)
  endif()
  if(CONFIG_ESPRESSIF_I2S)
    list(APPEND SRCS esp32s3_board_i2s.c)
  endif()
  if(CONFIG_AUDIO_CS4344)
    list(APPEND SRCS esp32s3_cs4344.c)
  endif()
  if(CONFIG_AUDIO_ES8311)
    list(APPEND SRCS esp32s3_es8311.c)
  endif()
  if(CONFIG_SENSORS_BMP180)
    list(APPEND SRCS esp32s3_board_bmp180.c)
  endif()
  if(CONFIG_SENSORS_LSM6DS3TRC)
    list(APPEND SRCS esp32s3_board_lsm6ds3trc.c)
  endif()
  if(CONFIG_ESP32S3_OTG)
    list(APPEND SRCS esp32s3_board_usb.c)
  endif()
  if(CONFIG_ESPRESSIF_ADC)
    list(APPEND SRCS esp32s3_board_adc.c)
  endif()
  if(CONFIG_SENSORS_QMI8658)
    list(APPEND SRCS esp32s3_qmi8658.c)
  endif()
  if(CONFIG_NET_LAN9250)
    list(APPEND SRCS esp32s3_lan9250.c)
  endif()
  if(CONFIG_ESP_PCNT)
    list(APPEND SRCS esp32s3_board_pcnt.c)
  endif()
  if(CONFIG_ESP_RMT)
    list(APPEND SRCS esp32s3_board_rmt.c)
  endif()
  if(CONFIG_ESP_MCPWM)
    list(APPEND SRCS esp32s3_board_mcpwm.c)
  endif()
  if(CONFIG_MMCSD_SPI OR CONFIG_ESP32S3_SDMMC)
    list(APPEND SRCS esp32s3_board_sdmmc.c)
  endif()
  if(CONFIG_USBMSC)
    list(APPEND SRCS esp32s3_usbmsc.c)
  endif()
  if(CONFIG_ESPRESSIF_LEDC)
    list(APPEND SRCS esp32s3_board_ledc.c)
  endif()

  if(CONFIG_ETC_ROMFS)
    set(RCSRCS etc/init.d/rc.sysinit etc/init.d/rcS)
    if(CONFIG_SYSTEM_NXINIT)
      list(APPEND RCSRCS etc/init.d/init.rc)
    endif()
    if(CONFIG_TXTABLE_PARTITION)
      set(RCRAWS ${NUTTX_BOARD_DIR}/src/etc/txtable.txt)
    endif()
    nuttx_add_romfs(
      NAME
      etc
      MOUNTPOINT
      etc
      RCSRCS
      ${RCSRCS}
      RCRAWS
      ${RCRAWS}
      PATH
      ${CMAKE_CURRENT_BINARY_DIR}/etc)
    target_link_libraries(board PRIVATE romfs_etc)
  endif()
endif()

# The board's own headers, as Make's include path (arch/<arch>/src/board/src)

target_include_directories(board PRIVATE ${NUTTX_BOARD_DIR}/src)

target_sources(board PRIVATE ${SRCS})
