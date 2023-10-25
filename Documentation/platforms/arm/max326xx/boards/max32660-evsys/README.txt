README
======

  This directory holds NuttX board support for the Maxim Integrated
  MAX32660-EVSYS board.  That board features:

  o MAX32660 Microcontroller
    - Arm Cortex-M4F, 96MHz
    - 256KB Flash Memory
    - 96KB SRAM
    - 16KB Instruction Cache
    - Two SPIs
    - Two I2Cs
    - Two UARTs
    - 14 GPIOs
  o DIP Breakout Board
    - 100mil Pitch Dual Inline Pin Headers
    - Breadboard Compatible
  o Integrated Peripherals
    - Red Indicator LED
    - User Pushbutton
  o MAX32625PICO-Based Debug Adapter
    - CMSIS-DAP SWD Debugger
    - Virtual UART Console

Contents
========

  o Status
  o Serial Console
  o LEDs and Buttons
  o OpenOCD

Status
======

  2018-11-21:  The port is code complete but completely untested.  I am
    still waiting to receive hardware to perform the bringup.  This initial
    port will support an NSH console:  Clock configuration, timer, GPIO
    pin configuration, ICC, and UART.  Additional untested drivers are
    complete and ready for testing:  DMA, GPIO interrupts, RTC, WDT.  The
    following drivers are not implemented: I2C, SPI, I2S.
  2018-11-27:  I received the MAX32660-EVSYS today and made a little debug
    progress.  Added a run-from-SRAM configuration to keep from locking
    up the board on bad configurations.  The rest of the bring-up will
    use this SRAM configuration.
    The NSH prompt comes up, but there is some interrupt-related problem
    that makes the console unusable.
  2018-11-28:  Found that the WFI instruction in the IDLE loop was causing
    instability.  System ran OK until it was in IDLE then it became
    unstable.  Commenting out the WIF restores stability.  The port now
    runs safely from FLASH although still with missing UART interrupts.
    Also fixed the on-board LED which now currently reflects the state.
  2018-11-29:  Resolved the UART interrupt issue.  The NSH configuration
    now appears fully functional.  Removed EXPERIMENTAL from configuration.
    Brought in the STM32 SPI driver as a starting point.  It still does
    not build correctly.  Due to conflicts, only SPI0 will be available.
  2018-11-30:  Completed coding of the SPI driver.  Added board support
    for SPI and for and SPI-based micro-SD card.  Initial testing with no
    device attached shows that the first single byte SPI transfer hangs
    with 1 byte in the Tx FIFO and nothing in the Rx FIFO.  Data is not
    moving.

  I need to stop and work on some other things for a while.  Here are the
  things remaining to be done:

  - Test the DMA, WDT and RTC drivers.  These drivers are complete but
    untested.
  - Finish verification of the SPI0 master driver.  It currently hangs as
    described above.
  - Add support for missing drivers.

  Missing drivers include:

  - SPI17Y:  SPI0 master DMA support, SPI0 Slave
  - SPIMSS: SPI1 master/slave, I2C
  - I2C0/1 master/slave
  - Timer/PWM

Serial Console
==============

  UART1 Tx and Rx signals at port P0.10 and P0.11 are connected to the
  programming and debug header JH2 pins 2 and 3 through 1kΩ resistors.
  This provides a convenient way to communicate with a PC though the
  virtual serial port available in Maxim’s CMSIS-DAP debug adapter. The
  series resistors allow for these signals to be overdriven by other
  circuits without modifying the board.

LEDs and Buttons
================

LEDs
----

  A single red LED is available driven by GPIO P0.13.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_autoleds.c. The LED is used to encode
  OS-related events as follows:

    ------------------- ----------------------- ------
    SYMBOL              Meaning                 LED
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus if the LED is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If the LED is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

Buttons
-------

  An single button is available on GPIO P0.12 for use by software.

OpenOCD
=======

Starting OpenOCD
----------------

  An Eclipse based toolchain is available for download from Maxim Integrated.
  If you (like me) are not an IDE user then the good news is the OpenOCD for
  the MAX32660 is available within that toolchain.

  As of this writing, the OpenOCD changes for the MAX32660 are not yet
  incorporated into the mainline OpenOCD code so the Maxim Integrated version
  within the Eclipse-based toolchain is the only show in town.  Patches for
  the MAX32660 have been submitted and this will most likely no longer be true
  as you read this.

  The Eclipse-based toolchain installs by default at C:\Maxim under Windows.
  The following script tracks tracks down OpenOCD in that installation
  (assuming Cygwin, hence the /cygdrive/c for the C: drive:

    #!/bin/sh

    set -x

    OPENOCD=/cygdrive/c/Maxim/Toolchain/bin/openocd.exe
    IFCFG="C:\Maxim\Toolchain\share\openocd\scripts\interface\max32660_hdk.cfg"
    MCUCFG="C:\Maxim\Toolchain\share\openocd\scripts\target\max32660.cfg"

    ${OPENOCD} ${1} -f ${IFCFG} # -f ${MCUCFG}

Loading Code:

  Code can be loaded into FLASH using the convenient ARM MBED drag'n'drop
  interface.  Or it can be loaded into FLASH (or SRAM) using GDB as follows:

  $ arm-none-eabi-gdb
  (gdb) target remote localhost:3333
  (gdb) mon reset
  (gdb) mon halt
  (gdb) load nuttx

  This does not work so reliably for me, however.

Debugging from FLASH:

  $ arm-none-eabi-gdb
  (gdb) target remote localhost:3333
  (gdb) mon reset
  (gdb) mon reg pc __start  # Set PC to __start entry point
  (gdb) file nuttx
  (gdb) b nx_start
  (gdb) c

  Also not very reliable.

Debugging from SRAM:

  Same except (1) that the __start entry point is 0x2000011c, not 0x11c, and
  the code needs to be reloaded into SRAM each time:

  (gdb) target remote localhost:3333
  (gdb) mon reset
  (gdb) mon halt
  (gdb) load nuttx          # Re-load code into SRAM
  (gdb) mon reg pc __start  # Set PC to __start entry point
  (gdb) file nuttx
  (gdb) b nx_start
  (gdb) c

Recovering from bad code in FLASH:

  In my initial debug effort, I had a lethal bug that I thought had bricked
  the board.  It appears that initialization logic put the MAX32660 in a bad
  state so that every time that I reset the board, I would re-enter this
  same bad state and I could not connect the CMSIS-DAP debugger.

  I was able to recover.   I jumpered GND to RSTn the used the MBED MSC
  interface to copy a known safe 'SysTick' demo program.  The copy hung and
  timed out with an error message.  I yanked the jumper off RSTn and asked
  to re-try copy.  It continued to burn the safe code demo!  I fixed it!
  too much drama.
