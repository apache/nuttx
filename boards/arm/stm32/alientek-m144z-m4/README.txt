README
======

This README discusses issues unique to NuttX configurations for the
ALIENTEK M144Z-M4 board (also marketed as "STM32F407 minimum system
board") populated with an STMicroelectronics STM32F407ZGT6 part:

  Chip                ARM Cortex-M4F @ 168 MHz
  FLASH               1024 KiB
  SRAM                192 KiB (128 KiB SRAM1 + 64 KiB SRAM2/CCM)
  Package             LQFP144

The board ships with:

  - On-board CH340 USB-UART (USART1 PB6/PB7, default NSH console)
  - W25Q128 16 MiB SPI NOR flash on SPI1 (CS = PB14)
  - Two user LEDs, active LOW
        LED0 (red)   = PF9
        LED1 (green) = PF10
  - Two user buttons (EXTI-capable)
        KEY0  = PE4   (pull-up, active LOW, BAT54C tied to BOOT0)
        WK_UP = PA0   (pull-down, active HIGH, doubles as wakeup)
  - USB OTG-FS Type-C connector wired to PA11/PA12
  - 8 MHz HSE crystal, 32.768 kHz LSE crystal
  - 5 V barrel jack power input with on-board 5V->3V3 regulator

Contents
========

  - Serial Console
  - LEDs
  - Buttons
  - On-board W25Q128 SPI flash
  - Configurations

Serial Console
==============

By default NSH runs on USART1 (PB6=TX, PB7=RX, 115200-8N1).  The on-board
CH340 USB-UART exposes USART1 as a virtual COM port on the host PC, so
no external adapter is required.

LEDs
====

If CONFIG_ARCH_LEDS is selected the kernel uses LED0 (red, PF9) as the
status LED via boards/arm/stm32/alientek-m144z-m4/src/stm32_autoleds.c:

    LED_STARTED       OFF
    LED_STACKCREATED  ON
    LED_PANIC         Blinking

Otherwise both LEDs are available to the application through the
userled driver (/dev/userleds).

Buttons
=======

When CONFIG_INPUT_BUTTONS is enabled the two on-board buttons are
exposed at /dev/buttons.  Both pins are also EXTI-capable; the WK_UP
button doubles as the STM32 standby-mode wakeup source.

On-board W25Q128 SPI flash
==========================

The on-board Winbond W25Q128 16 MiB NOR flash is wired to SPI1 with
GPIO chip-select on PB14.  When CONFIG_MTD_W25=y the bring-up code
in stm32_bringup.c probes the part (reading manufacturer ID and unique
ID through SPI1) and exposes it as the raw MTD character device
/dev/w25.  When CONFIG_FS_LITTLEFS is additionally enabled a littlefs
filesystem is auto-mounted at /data on first boot.

Configurations
==============

  nsh
  ---
    Basic NuttShell configuration; USART1 console, two LEDs, two buttons,
    W25Q128 SPI flash exposed as /dev/w25, littlefs auto-mounted at /data.
    Built with the ARM EABI toolchain.

  Sample build session:

    $ tools/configure.sh alientek-m144z-m4:nsh
    $ make

  The resulting binaries can be programmed over the on-board ST-Link
  compatible SWD header or via the USB-DFU bootloader entered by holding
  KEY0 (which is wired to BOOT0 through a BAT54C diode) during reset.
