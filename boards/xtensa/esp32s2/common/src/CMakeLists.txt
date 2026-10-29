# ##############################################################################
# boards/xtensa/esp32s2/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ESPRESSIF_ADC)
    list(APPEND SRCS esp32s2_board_adc.c)
  endif()
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32s2_board_wdt.c)
  endif()
  if(CONFIG_ESP32S2_SPI)
    list(APPEND SRCS esp32s2_board_spi.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32s2_board_spidev.c)
  endif()
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp32s2_board_wlan.c)
  endif()
  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp32s2_board_spislavedev.c)
  endif()
  if(CONFIG_ESPRESSIF_SPIFLASH)
    list(APPEND SRCS esp32s2_board_spiflash.c)
  endif()
  if(CONFIG_SENSORS_MAX6675)
    list(APPEND SRCS esp32s2_max6675.c)
  endif()
  if(CONFIG_ESPRESSIF_I2S)
    list(APPEND SRCS esp32s2_board_i2sdev.c)
  endif()
  if(CONFIG_ESP32S2_TWAI)
    list(APPEND SRCS esp32s2_board_twai.c)
  endif()
  if(CONFIG_AUDIO_CS4344)
    list(APPEND SRCS esp32s2_cs4344.c)
  endif()
  if(CONFIG_LCD_ILI9341)
    list(APPEND SRCS esp32s2_ili9341.c)
  endif()
  if(CONFIG_LCD_ST7789)
    list(APPEND SRCS esp32s2_st7789.c)
  endif()
  if(CONFIG_AUDIO_ES8311)
    list(APPEND SRCS esp32s2_es8311.c)
  endif()
  if(CONFIG_ESP_RMT)
    list(APPEND SRCS esp32s2_board_rmt.c)
  endif()
  if(CONFIG_ESP_PCNT)
    list(APPEND SRCS esp32s2_board_pcnt.c)
  endif()
  if(CONFIG_MMCSD_SPI)
    list(APPEND SRCS esp32s2_board_sdmmc.c)
  endif()

  if(CONFIG_ESP32S2_TWAI
     OR CONFIG_ARCH_BUTTONS
     OR CONFIG_DEV_GPIO)
    string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")
    set(CHIPHALDIR
        ${NUTTX_BINARY_DIR}/arch/xtensa/src/common/espressif/esp-hal-3rdparty)
    target_compile_options(board PRIVATE -Wno-undef)
    target_include_directories(
      board
      PRIVATE ${CHIPHALDIR}/nuttx/${CHIP_SERIES}/include
              ${CHIPHALDIR}/components/esp_common/include
              ${CHIPHALDIR}/components/soc/${CHIP_SERIES}/include
              ${CHIPHALDIR}/components/soc/${CHIP_SERIES}/register)
  endif()

  if(CONFIG_ETC_ROMFS)
    nuttx_add_romfs(
      NAME
      etc
      MOUNTPOINT
      etc
      RCSRCS
      etc/init.d/rc.sysinit
      etc/init.d/rcS
      PATH
      ${CMAKE_CURRENT_BINARY_DIR}/etc)
    target_link_libraries(board PRIVATE romfs_etc)
  endif()
endif()

# The board's own headers, as Make's include path (arch/<arch>/src/board/src)

target_include_directories(board PRIVATE ${NUTTX_BOARD_DIR}/src)

target_sources(board PRIVATE ${SRCS})
