README
======

This is the README file for a port of NuttX to the ST Micro B-G474E-DPOW1
Discovery kit with STM32G474RE MCU. For more information about this board,
see:

  https://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-discovery-kits/b-g474e-dpow1.html


Contents
========

  - Status
  - Development Environment
    - Toolchains
    - Debugging
  - Hardware
    - MCU Clocking
    - GPIOs
    - Buttons
    - LEDs
    - RGB Power LED
  - Serial Consoles
  - Configurations


Status
======

  This port boots NuttX through to a functional NSH prompt.


Development Environment
=======================

  Toolchains
  ----------
  An appropriate ARM toolchain is needed, such as the one built with the
  customized NuttX buildroot or the ready-made GNU Tools for Arm Embedded
  Processors.


  Debugging
  ---------
  The board incorporates a STLINK-V3E programmer/debugger accessible via the
  Micro-USB Type B connector.

  To debug with OpenOCD and arm-nuttx-eabi-gdb:

  * Use 'make menuconfig' to set CONFIG_DEBUG_SYMBOLS and CONFIG_DEBUG_NOOPT.
    To see debug output, e.g., the "ABCDE" printed in __start(), also set
    CONFIG_DEBUG_FEATURES.

  * Build NuttX.

  * Flash the code using:
    $ openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c init \
      -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"

  * Start GDB with:
    $ arm-nuttx-eabi-gdb -tui nuttx

  * In GDB:
    (gdb) target remote localhost:3333
    (gdb) monitor reset halt
    (gdb) load


Hardware
========

  MCU Clocking
  ------------
  By default, the MCU on this board is clocked from the MCU's internal HSI
  clock, and only this option is supported by software at this time.

  If software support is added for it, the MCU could be clocked from the
  following other sources: a 24 MHz oscillator on X2, MCO from STLINK-V3E, or
  external clock from connector CN9, pin 26.


  GPIOs
  -----


  Buttons
  -------
  The board has 5 user buttons in the form of a 4-direction "joystick" with a
  selection button (pressing down on the "joystick").


  LEDs
  ----
  The board has 4 user LEDs.


  RGB Power LED
  -------------
  The board has a super bright RGB power LED.

  Caution: For eye safety, ensure that the power LED is covered by the
  diffuser that comes installed over it.


Serial Consoles
===============

  The MCU's USART3 is connected to the on-board STLINK-V3E and exposed to
  the PC as a Virtual COM Port over the same Micro-USB Type B connection used
  for programming/debugging.

  On Debian Linux, this shows up as /dev/ttyACM0. Other operating systems may
  differ.


Configurations
==============

  nsh
  ---

