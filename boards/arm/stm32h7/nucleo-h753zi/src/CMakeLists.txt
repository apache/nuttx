# ##############################################################################
# boards/arm/stm32h7/nucleo-h753zi/src/CMakeLists.txt
# ##############################################################################

set(SRCS stm32_boot.c stm32_bringup.c stm32_appinitialize.c)

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if(CONFIG_STM32H7_I2C)
  list(APPEND SRCS stm32_i2c.c)
endif()

if(CONFIG_STM32H7_SPI)
  list(APPEND SRCS stm32_spi.c)
endif()

if(CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE)
  list(APPEND SRCS stm32_mfrc522.c)
endif()

if(CONFIG_STM32H7_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(CONFIG_SENSORS_LSM6DSL)
  list(APPEND SRCS stm32_lsm6dsl.c)
endif()

if(CONFIG_SENSORS_LSM9DS1)
  list(APPEND SRCS stm32_lsm9ds1.c)
endif()

if(CONFIG_SENSORS_LSM303AGR)
  list(APPEND SRCS stm32_lsm303agr.c)
endif()

if(CONFIG_PCA9635PW)
  list(APPEND SRCS stm32_pca9635.c)
endif()

if(CONFIG_LCD_SSD1306)
  list(APPEND SRCS stm32_ssd1306.c)
endif()

if(CONFIG_LCD_ST7796)
  list(APPEND SRCS stm32_st7796.c)
endif()

if(CONFIG_STM32H7_PROGMEM)
  list(APPEND SRCS stm32_progmem.c)
endif()

if(CONFIG_WL_NRF24L01)
  list(APPEND SRCS stm32_nrf24l01.c)
endif()

if(CONFIG_NUCLEO_H753ZI_GPIO_DRIVER)
  list(APPEND SRCS stm32_gpio.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS stm32_reset.c)
endif()

if(CONFIG_BOARDCTL_BOOT_IMAGE)
  list(APPEND SRCS stm32_boot_image.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

if(CONFIG_MMCSD)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

target_sources(board PRIVATE ${SRCS})

# Linker scripts
if(CONFIG_STM32_APP_FORMAT_MCUBOOT)
  if(CONFIG_MCUBOOT_BOOTLOADER)
    set_property(
      GLOBAL PROPERTY LD_SCRIPT
                      "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-loader.ld")
  else()
    set_property(
      GLOBAL PROPERTY LD_SCRIPT
                      "${NUTTX_BOARD_DIR}/scripts/flash-mcuboot-app.ld")
  endif()
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
endif()

if(NOT CONFIG_BUILD_FLAT)
  add_subdirectory(${NUTTX_BOARD_DIR}/kernel)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT_USER ${NUTTX_BOARD_DIR}/scripts/memory.ld
                    ${NUTTX_BOARD_DIR}/scripts/user-space.ld)
endif()
