==============
mikroe-stm32f4
==============

This README discusses issues unique to NuttX configurations for the
MikroElektronika Mikromedia for STM32F4 development board.  This is
another board support by NuttX that uses the same STM32F407VGT6 MCU
as does the STM32F4-Discovery board. This board, however, has very
different on-board peripherals than does the STM32F4-Discovery:

- TFT display with touch panel,
- VS1053 stereo audio codec with headphone jack,
- SD card slot,
- Serial FLASH memory,
- USB OTG FS with micro-AB connector, and
- Battery connect and batter charger circuit.

See the http://www.mikroe.com/mikromedia/stm32-m4/ for more information
about this board.

LEDs
====

The Mikroe-STM32F4 board has no user accessible LEDs onboard, only a power
and "charging" LED.  All visual user output must be performed through the TFT
display.

External LEDs could be added via the expansion headers on the side of the
board, but as this would be a custom configuration, LEDs are not supported
in this port.

PWM
===

The Mikroe-STM32F4 has no real on-board PWM devices, but it does have PWM
pins routed to the expansion I/O headers on the side of the board.

UARTs
=====

The Mikroe-STM32F4 board has no onboard RS-232 line driver, however the
expansion I/O header provides access to USART2 on pins PD5/PD6.  The port
includes support for USART2 configured as /dev/ttyS0.

USART2
------

========== =====
UART/USART PINS
========== =====
RX         PD6
TX         PD5
========== =====

Default USART/UART Configuration
--------------------------------

USART2 is enabled in all configurations (see \*/defconfig).  RX and TX are
configured on pins PD6 and PD5, respectively (see include/board.h).

Timer Inputs/Outputs
====================

::
   TIM1
     CH1     PA8, PE9
     CH2     PA9[1], PE11
     CH3     PA10[1], PE13
     CH4     PA11[1], PE14
   TIM2
     CH1     PA0[1], PA15, PA5[1]
     CH2     PA1, PB3[1]
     CH3     PA2, PB10[1]
     CH4     PA3, PB11
   TIM3
     CH1     PA6[1], PB4, PC6
     CH2     PA7[1], PB5, PC7[1]
     CH3     PB0, PC8
     CH4     PB1, PC9
   TIM4
     CH1     PB6[1], PD12[1]
     CH2     PB7, PD13[1]
     CH3     PB8, PD14[1]
     CH4     PB9[1], PD15[1]
   TIM5
     CH1     PA0[1], PH10[2]
     CH2     PA1, PH11[2]
     CH3     PA2, PH12[2]
     CH4     PA3, PI0
   TIM8
     CH1     PC6, PI5
     CH2     PC7[1], PI6
     CH3     PC8, PI7
     CH4     PC9, PI2
   TIM9
     CH1     PA2, PE5
     CH2     PA3, PE6
   TIM10
     CH1     PB8, PF6
   TIM11
     CH1     PB9[1], PF7
   TIM12
     CH1     PH6[2], PB14
     CH2     PC15, PH9[2]
   TIM13
     CH1     PA6[1], PF8
   TIM14
     CH1     PA7[1], PF9

   [1] Indicates pins that have other on-board functions and should be used only
   with care (See table 5 in the Mikroe-STM32F4 User Guide).  The rest are
   free I/O pins.
   [2] Port H pins are not supported by the MCU

MIO283QT-2/MIO283QT-9A
======================

The original Mikroe-SMT32F4 board as an on-board MIO283QT-2 TFT LCD that can
be configured and used.  This is a 320x240 resolution display with color
capability to 262K colors, though the mio283qt-2 driver in NuttX only
supports 16-bit color depth, or 65K colors.  Changes to both the
mio283qt-2 driver and the driver interface layer would need to be made
to support 24 BPP mode.

UPDATE:  New boards now support a MIO283QT-9A TFT LCD that is not compatible
with the MIO283QT-2.  It uses a different LCD controller.  The default in
all of these configurations is the MIO283QT-2.  But MIO283QT-9A is also
supported and you can switch from the MIO283QT-2 to the MIO283QT-9A by simply
modifying the NuttX configuration

Configurations
==============

Each Mikroe-STM32F4 configuration is maintained in a sub-directory and
can be selected as follow::

    tools/configure.sh mikroe-stm32f4:<subdir>

If this is a Windows native build, then configure.bat should be used
instead of configure.sh::

    configure.bat Mikroe-STM32F4\<subdir>

Where <subdir> is one of the following:

fulldemo
--------

This is an example that includes an NSH shell over USB that also
enables all features of the Mikroe-STM32F4 board including the LCD,
on-board 1M Flash with SMART filesystem, Aux RS-232 serial port on the
expansion header, etc.  A couple of the NX graphics commands are made
available via the NSH prompt for performing LCD demonstrations, and the
nximage example is used as a splash-screen at startup.

kostest
-------

NOTE: This configuration compiles, but has not been fully tested
on the hardware yet.

This configuration directory, performs a simple OS test using
apps/examples/ostest with NuttX build as a kernel-mode monolithic
module and the user applications are built separately.  Is
is recommended to use a special make command; not just 'make' but
make with the following two arguments::

        make pass1 pass2

In the normal case (just 'make'), make will attempt to build both user-
and kernel-mode blobs more or less interleaved.  This actual works!
However, for me it is very confusing so I prefer the above make command:
Make the user-space binaries first (pass1), then make the kernel-space
binaries (pass2)

NOTES:

1. This configuration uses the mconf-based configuration tool.  To
   change this configuration using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

2. This is the default platform/toolchain in the configuration::

       CONFIG_HOST_WINDOWS=y                   : Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Cygwin environment on Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

       This is easily changed by modifying the configuration.

3. At the end of the build, there will be several files in the top-level
   NuttX build directory::

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

4. Combining .hex files.  If you plan to use the STM32 ST-Link Utility to
   load the .hex files into FLASH, then you need to combine the two hex
   files into a single .hex file.  Here is how you can do that.

   a. The 'tail' of the nuttx.hex file should look something like this
      (with my comments added)::

            $ tail nuttx.hex
            # 00, data records
            ...
            :10 9DC0 00 01000000000800006400020100001F0004
            :10 9DD0 00 3B005A0078009700B500D400F300110151
            :08 9DE0 00 30014E016D0100008D
            # 05, Start Linear Address Record
            :04 0000 05 0800 0419 D2
            # 01, End Of File record
            :00 0000 01 FF

          Use an editor such as vi to remove the 05 and 01 records.

   b. The 'head' of the nuttx_user.hex file should look something like
      this (again with my comments added)::

            $ head nuttx_user.hex
            # 04, Extended Linear Address Record
            :02 0000 04 0801 F1
            # 00, data records
            :10 8000 00 BD89 01084C800108C8110208D01102087E
            :10 8010 00 0010 00201C1000201C1000203C16002026
            :10 8020 00 4D80 01085D80010869800108ED83010829
            ...

          Nothing needs to be done here.  The nuttx_user.hex file should
          be fine.

   c. Combine the edited nuttx.hex and un-edited nuttx_user.hex
      file to produce a single combined hex file::

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the STM32 ST-Link tool.  If
       you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

nsh
---

This is an NSH example that uses USART2 as the console.  Note that
the Mikroe-STM32F4 board doesn't actually have onboard line drivers
or a connector for USART2, but it does route the USART2 signals to
the expansion header.  To use this demo, you would need to connect
an external 3.3V RS-232 line driver to the USART's I/O lines on the
expansion header.

NOTE:  This demo doesn't quite work yet.  I can get output to the
USART, but so far, I have not gotten nsh to actually come up.

nx
--

An example using the NuttX graphics system (NX).  This example
focuses on general window controls, movement, mouse and keyboard
input.::

      CONFIG_LCD_LANDSCAPE=y        : 320x240 landscape orientation
      CONFIG_LCD_MIO283QT2=y        : MIO283QT-2 is the default

You can the newer  MIO283QT-9A by enabling it in the configuration.::

      CONFIG_LCD_MIO283QT2=n         : Disable the MIO283QT-2
      CONFIG_LCD_MIO283QT9A=y        : Enable the MIO283QT-9A

nxlines
-------

An example using the NuttX graphics system (NX).   This example focuses on
placing lines on the background in various orientations using the
on-board TFT LCD.::

      CONFIG_LCD_LANDSCAPE=y        : 320x240 landscape orientation
      CONFIG_LCD_MIO283QT2=y        : MIO283QT-2 is the default

You can the newer  MIO283QT-9A by enabling it in the configuration.::

      CONFIG_LCD_MIO283QT2=n         : Disable the MIO283QT-2
      CONFIG_LCD_MIO283QT9A=y        : Enable the MIO283QT-9A

nxtext
------

Another example using the NuttX graphics system (NX).   This
example focuses on placing text on the background while pop-up
windows occur.  Text should continue to update normally with
or without the popup windows present.

usbnsh
-------

This is another NSH example.  If differs from other 'nsh' configurations
in that this configurations uses a USB serial device for console I/O.
Such a configuration is useful on the stm32f4discovery which has no
builtin RS-232 drivers.

NOTES:

1. This configuration uses the mconf-based configuration tool.  To
   change this configuration using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.

2. By default, this configuration uses the ARM EABI toolchain
   for Windows and builds under Cygwin (or probably MSYS).  That
   can easily be reconfigured, of course.::

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

3. This configuration does have UART2 output enabled and set up as
   the system logging device::

       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART2 will be /dev/ttyS0

     However, there is nothing to generate SYSLOG output in the default
     configuration so nothing should appear on UART2 unless you enable
     some debug output or enable the USB monitor.

4. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
   device will save encoded trace output in in-memory buffer; if the
   USB monitor is enabled, that trace buffer will be periodically
   emptied and dumped to the system logging device (UART2 in this
   configuration)::

       CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
       CONFIG_USBDEV_TRACE_NRECORDS=128        : Buffer 128 records in memory
       CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
       CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor
       CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
       CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
       CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
       CONFIG_USBMONITOR_INTERVAL=2     : Dump trace data every 2 seconds

       CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
       CONFIG_USBMONITOR_TRACECLASS=y
       CONFIG_USBMONITOR_TRACETRANSFERS=y
       CONFIG_USBMONITOR_TRACECONTROLLER=y
       CONFIG_USBMONITOR_TRACEINTERRUPTS=y

5. By default, this project assumes that you are *NOT* using the DFU bootloader.

Using the Prolifics PL2303 Emulation
------------------------------------

You could also use the non-standard PL2303 serial device instead of
the standard CDC/ACM serial device by changing::

      CONFIG_CDCACM=y               : Disable the CDC/ACM serial device class
      CONFIG_CDCACM_CONSOLE=y       : The CDC/ACM serial device is NOT the console
      CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
      CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console
