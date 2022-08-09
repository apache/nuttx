Nucleo-WL55JC README
====================

  This README file discusses the port of NuttX to the STMicro Nucleo-WL55JC
  board. That board features the STM32WL55JCI7 MCU with 256KiB of FLASH and
  64KiB of SRAM. This is dual CPU (not core) chip. There is integrated LORA
  hardware on board which is only available via CPU0.

Contents
========

  - Status
  - LEDs
  - Buttons
  - Serial Console
  - Configurations
  - Flashing

Status
======

  2022.06.07: Board boots and nsh works without problems. Both arduino and
  virtual com port UARTs work.

LEDs
====

  There are user controlled 3 LEDs. Blue (PB15), Green(PB9) and Red(PB11).
  To turn on the LED, GPIO has to be driven to HIGH state.

  Green and Red LEDs are used by the system at boot to show system state.
  Once system is booted these LEDs are for user to control. When
  CONFIG_ARCH_LEDS is set, Blue LED is reserved by OS for reporting system
  status. When CONFIG_ARCH_LEDS is not set, OS state won't be reported on
  any of the LEDs and all 3 LEDs are available for user right from the start.

Buttons
=======

  There are 3 buttons that are available for the user to program, and one
  reset button.

Serial Console
==============

  There are 2 serial ports - USART1 and LPUART1.

  USART1 is connected to arduino D0/D1 pin and LPUART is connected to
  stlink that provides virtual serial port.

  NSH is configured to use LPUART and virtual serial port. After flashing
  you can open /dev/ttyACM0 (may change depending on your system) and nsh
  prompt will be waiting for you there. Serial device does not disapear
  when flashing and reseting board - it can be left opened and flashing
  will work without problems.

Configuration
=============

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at examples/nsh. NSH is will
    work on virtual serial port over usb.

Flashing
========

  Easiest way to flash nucleo is to use openocd tool. Openocd supports
  stlink v3 which is on the board. It's as easy as running:

  openocd -f interface/stlink.cfg -f target/stm32wlx.cfg \
      -c "program nuttx.bin exit 0x08000000"
