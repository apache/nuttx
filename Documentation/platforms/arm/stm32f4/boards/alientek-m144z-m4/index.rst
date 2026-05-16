=================
ALIENTEK M144Z-M4
=================

.. tags:: chip:stm32, chip:stm32f4, chip:stm32f407

The ALIENTEK M144Z-M4 (also sold as "STM32F407 minimum system board") is a
compact development board built around the STMicroelectronics STM32F407ZGT6
(ARM Cortex-M4F, 168 MHz, 1 MiB flash, 192 KiB SRAM, LQFP144).

Features used by this BSP:

- On-board CH340 USB-UART on USART1 (PB6/PB7) -- default NSH console
- Winbond W25Q128 16 MiB SPI NOR flash on SPI1, CS = PB14
- 2x user LEDs (active LOW): LED0 = PF9 (red), LED1 = PF10 (green)
- 2x user buttons (EXTI-capable): KEY0 = PE4, WK_UP = PA0
- USB OTG-FS on PA11/PA12 (Type-C connector, no VBUS sensing)
- 8 MHz HSE crystal, 32.768 kHz LSE crystal

Serial Console
==============

USART1 is wired to the on-board CH340 USB-UART:

  ==========  ====
  STM32 pin   Net
  ==========  ====
  PB6         TX
  PB7         RX
  ==========  ====

Default settings: 115200-8N1.

LEDs
====

The board has two user LEDs::

    LED0 (red)   = PF9
    LED1 (green) = PF10

Both are active LOW (510 R series resistor to 3V3, driven LOW to light).

If ``CONFIG_ARCH_LEDS`` is selected the kernel claims LED0 as a status
indicator via ``stm32_autoleds.c``:

  ================  ========
  Event             LED0
  ================  ========
  LED_STARTED       OFF
  LED_STACKCREATED  ON
  LED_PANIC         Blinking
  ================  ========

Otherwise both LEDs are user-controllable through ``/dev/userleds``.

Buttons
=======

Two on-board buttons::

    KEY0  = PE4  -- active LOW (pull-up).  Shared with BOOT0 via a BAT54C
                    diode so holding KEY0 during reset enters the STM32
                    UART ISP / DFU bootloader.
    WK_UP = PA0  -- active HIGH (pull-down).  Doubles as the STM32
                    standby-mode wakeup source.

Both pins are EXTI-capable.  When ``CONFIG_INPUT_BUTTONS=y`` they are
exposed at ``/dev/buttons``.

On-board SPI Flash
==================

A Winbond W25Q128 (16 MiB NOR) is wired to SPI1 with chip select on PB14.
When ``CONFIG_MTD_W25=y`` ``stm32_bringup()`` probes the part (JEDEC ID +
unique ID dump in the boot log) and exposes it as ``/dev/w25``.  Adding
``CONFIG_FS_LITTLEFS=y`` mounts a littlefs filesystem at ``/data`` on the
same device with ``autoformat`` semantics, so the partition appears empty
the first time and persists across resets afterwards.

Configurations
==============

Each configuration is selected with ``tools/configure.sh``:

.. code-block:: console

    $ tools/configure.sh alientek-m144z-m4:<subdir>
    $ make

nsh
---

Basic NuttShell configuration.  USART1 console at 115200-8N1, both user
LEDs and buttons exposed, W25Q128 mounted as littlefs at ``/data``.
Compiles with the ARM EABI toolchain (``CONFIG_ARM_TOOLCHAIN_GNU_EABI``).

Programming
===========

The board can be programmed in three ways:

- **SWD** via the 4-pin SWD header (3V3 / SWCLK / SWDIO / GND) using an
  external ST-Link V2 / J-Link probe and OpenOCD or ``st-flash``.
- **USB-DFU** by holding KEY0 (= BOOT0 high) during reset and using
  ``dfu-util`` over the on-board Type-C connector.
- **Serial ISP** through the CH340 USB-UART using ``stm32flash`` after
  pulling BOOT0 high via KEY0.

Example (OpenOCD over ST-Link)::

    openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
            -c 'program nuttx.bin 0x08000000 verify reset exit'
