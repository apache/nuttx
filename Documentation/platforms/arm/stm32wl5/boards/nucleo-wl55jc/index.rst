=============
Nucleo-WL55JC
=============

The `Nucleo-WL55JC <https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html>`_
is a development board for the STM32WL55 SoC from ST. It features 64 I/O,
3 onboard LEDs and buttons, integrated stlink for easy debug and flashing
and on-board LoRa receiver with external antenna. NSH can be easily access
via virtual serial port from usb.

Features
========

  - STM32WL55JC MCU, 256K FLASH, 64K SRAM
  - 32768 Hz LSE crystal
  - 32 MHz HSE crystal
  - Embedded stlink-v3 debugger (debug/flash and virtual serial port)
  - Reset button
  - 3 user programmable LEDs
  - 3 user programmable buttons
  - Power indicator LED
  - LoRa radio with antenna
  - 64 Nucleo I/O
  - Arduino compatible pinout

Pin Mapping
===========

Pin mapping can be altered by (de)soldering bridges, by default board
uses following mapping:

===== ========== ============================
Pin   Signal     Notes
===== ========== ============================
PA2   LPUART1 TX Virtual serial port over usb
PA3   LPUART2 RX Virtual serial port over usb
PB6   USART1 TX  D1 on Arduino pinout
PB7   USART1 RX  R0 on Arduino pinout
PA0   Button 1
PA1   Button 2
PC6   Button 3
PB15  Blue LED   Active HIGH
PB11  Red LED    Active HIGH
PB9   Green LED  Active HIGH
===== ========== ============================

Default Peripherals Configuration
=================================

LED
---

Green and Red LEDs are used by the system at boot to show system state.
Once system is booted these LEDs are for user to control. When
CONFIG_ARCH_LEDS is set, Blue LED is reserved by OS for reporting system
status. When CONFIG_ARCH_LEDS is not set, OS state won't be reported on
any of the LEDs and all 3 LEDs are available for user right from the start.

Serial Console
--------------

There are 2 serial ports - USART1 and LPUART1.

USART1 is connected to arduino D0/D1 pin and LPUART is connected to
stlink that provides virtual serial port.

NSH is configured to use LPUART and virtual serial port. After flashing
you can open /dev/ttyACM0 (may change depending on your system) and nsh
prompt will be waiting for you there. Serial device does not disapear
when flashing and reseting board - it can be left opened and flashing
will work without problems.

Configurations
==============

nsh
---

Basic NuttShell configuration (console enabled in LPUART1, exposed via USB
connection, at 115200 bps 8n1).

Flash & Debug
=============

Both flashing and debugging are done using the embedded stlink-v3 debugger.
OpenOCD can be invoked in the following way to flash::

    openocd -f interface/stlink.cfg -f target/stm32wlx.cfg \
        -c "program nuttx.bin exit 0x08000000"

To debug attach openocd to stlink with command::

    openocd -f interface/stlink.cfg -f target/stm32wlx.cfg

start gdb::

    arm-none-eabi-gdb ./nuttx --tui

attach to gdb server::

    target remote localhost:3333

(optionally) reset program and start from the very beginning::

    monitor reset halt

Remember to generate debug symbol by setting CONFIG_DEBUG_SYMBOLS
and optionally (for more natural flow in gdb step) suppress optimization
by setting CONFIG_DEBUG_NOOPT.
