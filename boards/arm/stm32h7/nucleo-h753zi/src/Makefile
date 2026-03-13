############################################################################
# boards/arm/stm32h7/nucleo-h753zi/src/Makefile
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_bringup.c stm32_appinitialize.c

ifeq ($(CONFIG_ADC),y)
  CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
  CSRCS += stm32_autoleds.c
else
  CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
  CSRCS += stm32_buttons.c
endif

ifeq ($(CONFIG_STM32_ROMFS),y)
  CSRCS += stm32_romfs_initialize.c
endif

ifeq ($(CONFIG_STM32H7_I2C),y)
  CSRCS += stm32_i2c.c
endif

ifeq ($(CONFIG_STM32H7_SPI),y)
  CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_NUCLEO_H753ZI_MFRC522_ENABLE),y)
  CSRCS += stm32_mfrc522.c
endif

ifeq ($(CONFIG_STM32H7_OTGFS),y)
  CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
  CSRCS += stm32_uid.c
endif

ifeq ($(CONFIG_SENSORS_LSM6DSL),y)
  CSRCS += stm32_lsm6dsl.c
endif

ifeq ($(CONFIG_SENSORS_LSM9DS1),y)
  CSRCS += stm32_lsm9ds1.c
endif

ifeq ($(CONFIG_SENSORS_LSM303AGR),y)
  CSRCS += stm32_lsm303agr.c
endif

ifeq ($(CONFIG_PCA9635PW),y)
  CSRCS += stm32_pca9635.c
endif

ifeq ($(CONFIG_LCD_SSD1306),y)
  CSRCS += stm32_ssd1306.c
endif

ifeq ($(CONFIG_LCD_ST7796),y)
  CSRCS += stm32_st7796.c
endif

ifeq ($(CONFIG_STM32H7_PROGMEM),y)
  CSRCS += stm32_progmem.c
endif

ifeq ($(CONFIG_WL_NRF24L01),y)
  CSRCS += stm32_nrf24l01.c
endif

ifeq ($(CONFIG_NUCLEO_H753ZI_GPIO_DRIVER),y)
  CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_PWM),y)
  CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
  CSRCS += stm32_reset.c
endif

ifeq ($(CONFIG_BOARDCTL_BOOT_IMAGE),y)
  CSRCS += stm32_boot_image.c
endif

ifeq ($(CONFIG_USBMSC),y)
  CSRCS += stm32_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
  CSRCS += stm32_composite.c
endif

ifeq ($(CONFIG_MMCSD),y)
  CSRCS += stm32_mmcsd.c
endif

include $(TOPDIR)/boards/Board.mk
