================
STM32F3Discovery
================

This README discusses issues unique to NuttX configurations for the
STMicro STM32F3Discovery development board.

LEDs
====

The STM32F3Discovery board has ten LEDs.  Two of these are controlled by
logic on the board and are not available for software control::

  LD1 PWR:   red LED indicates that the board is powered.
  LD2 COM:   LD2 default status is red. LD2 turns to green to indicate that
           communications are in progress between the PC and the ST-LINK/V2.

And eight can be controlled by software::

  User LD3:  red LED is a user LED connected to the I/O PE9 of the
           STM32F303VCT6.
  User LD4:  blue LED is a user LED connected to the I/O PE8 of the
           STM32F303VCT6.
  User LD5:  orange LED is a user LED connected to the I/O PE10 of the
           STM32F303VCT6.
  User LD6:  green LED is a user LED connected to the I/O PE15 of the
           STM32F303VCT6.
  User LD7:  green LED is a user LED connected to the I/O PE11 of the
           STM32F303VCT6.
  User LD8:  orange LED is a user LED connected to the I/O PE14 of the
           STM32F303VCT6.
  User LD9:  blue LED is a user LED connected to the I/O PE12 of the
           STM32F303VCT6.
  User LD10: red LED is a user LED connected to the I/O PE13 of the
           STM32F303VCT6.

These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
events as follows:

  ===================  =======================  ==========================
  SYMBOL                Meaning                 LED state
  ===================  =======================  ==========================
  LED_STARTED          NuttX has been started   LD3 ON
  LED_HEAPALLOCATE     Heap has been allocated  LD4 ON
  LED_IRQSENABLED      Interrupts enabled       LD4 ON
  LED_STACKCREATED     Idle stack created       LD6 ON
  LED_INIRQ            In an interrupt          LD7 should glow
  LED_SIGNAL           In a signal handler      LD8 might glow
  LED_ASSERTION        An assertion failed      LD9 ON while handling the assertion
  LED_PANIC            The system has crashed   LD10 Blinking at 2Hz
  LED_IDLE             STM32 is is sleep mode   (Optional, not used)
  ===================  =======================  ==========================

Serial Console
==============

The STM32F3Discovery has no on-board RS-232 driver, however USART2 is
configuration as the serial console in all configurations that use a serial
console.

There are many options for USART2 RX and TX pins.  They configured to use
PA2 (TX) and PA3 (RX) for connection to an external serial device because of
the following settings in the include/board.h file::

  #define GPIO_USART2_RX GPIO_USART2_RX_2
  #define GPIO_USART2_TX GPIO_USART2_TX_2

This can be found on the board at::

  TX, PA2, Connector P1, pin 14
  RX, PA3, Connector P1, pin 11

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the STM32 port.

1. Non-Lazy Floating Point Register Save

   In this configuration floating point register save and restore is
   implemented on interrupt entry and return, respectively.  In this
   case, you may use floating point operations for interrupt handling
   logic if necessary.  This FPU behavior logic is enabled by default
   with::

     CONFIG_ARCH_FPU=y

2. Lazy Floating Point Register Save.

   An alternative implementation only saves and restores FPU registers only
   on context switches.  This means: (1) floating point registers are not
   stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file::

     CONFIG_ARCH_FPU=y

Debugging
=========

If you are going to use a debugger, you should make sure that the following
settings are selection in your configuration file::

  CONFIG_DEBUG_SYMBOLS=y     : Enable debug symbols in the build
  CONFIG_ARMV7M_USEBASEPRI=y : Use the BASEPRI register to disable interrupts

STM32 ST-LINK Utility
---------------------
For simply writing to FLASH, I use the STM32 ST-LINK Utility.  At least
version 2.4.0 is required (older versions do not recognize the STM32 F3
device).  This utility is available from free from the STMicro website.

OpenOCD
-------
I am told that OpenOCD will work with the ST-Link, but I have never tried
it.

https://github.com/texane/stlink
--------------------------------
This is an open source server for the ST-Link that I have never used.

It is also possible to use an external debugger such as the Segger JLink
(EDU or commercial models) provided:

1) The CN4 jumpers are removed to disconnect the on-board STLinkV2 from
   the STM32F3.

2) The appropriate (20 pin connector to flying wire) adapter is used to connect
   the debugger to the required pins on the expansion headers (see below).

   Note that the 1x6 header on the STLinkV2 side of the board labeled "SWD"
   is for the STLink micro (STM32F1) and is not connected to the STM32F3.

3) OpenOCD version 0.9.0 or later is used.  Earlier versions support either
   JTAG only or are buggy for SWD.

The signals used with external (SWD) debugging are::

   VREF (3V)
   GROUND (GND)
   SWCLK (PA14)
   SWIO (PA13)
   SWO (PB3)
   RESET (NRST)

Configurations
==============

Each STM32F3Discovery configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh STM32F3Discovery:<subdir>

Where <subdir> is one of the following:

nsh:
---
Configures the NuttShell (nsh) located at apps/examples/nsh.  The
Configuration enables the serial interfaces on USART2.  Support for
builtin applications is enabled, but in the base configuration no
builtin applications are selected (see NOTES below).

NOTES:

1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

2. By default, this configuration uses the ARM EABI toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

3. This configuration includes USB Support (CDC/ACM device)::

       CONFIG_STM32_USB=y            : STM32 USB device support
       CONFIG_USBDEV=y               : USB device support must be enabled
       CONFIG_CDCACM=y               : The CDC/ACM driver must be built
       CONFIG_NSH_BUILTIN_APPS=y     : NSH built-in application support must be enabled
       CONFIG_NSH_ARCHINIT=y         : To perform USB initialization

   The CDC/ACM example is included as two NSH "built-in" commands.::

       CONFIG_SYSTEM_CDCACM=y      : Enable apps/system/cdcacm

   The two commands are::

       sercon : Connect the serial device a create /dev/ttyACM0
       serdis : Disconnect the serial device.

   NOTE:  The serial connections/disconnections do not work as advertised.
   This is because the STM32F3Discovery board does not provide circuitry for
   control of the "soft connect" USB pullup.  As a result, the host PC
   does not know the USB has been logically connected or disconnected.  You
   have to follow these steps to use USB:

       1) Start NSH with USB disconnected
       2) enter to 'sercon' command to start the CDC/ACM device, then
       3) Connect the USB device to the host.

   and to close the connection:

       4) Disconnect the USB device from the host
       5) Enter the 'serdis' command

4. This example can support the watchdog timer test (apps/examples/watchdog)
   but this must be enabled by selecting::

       CONFIG_EXAMPLES_WATCHDOG=y : Enable the apps/examples/watchdog
       CONFIG_WATCHDOG=y          : Enables watchdog timer driver support
       CONFIG_STM32_WWDG=y        : Enables the WWDG timer facility, OR
       CONFIG_STM32_IWDG=y        : Enables the IWDG timer facility (but not both)

   The WWDG watchdog is driven off the (fast) 42MHz PCLK1 and, as result,
   has a maximum timeout value of 49 milliseconds.  For WWDG watchdog, you
   should also add the following to the configuration file::

       CONFIG_EXAMPLES_WATCHDOG_PINGDELAY=20
       CONFIG_EXAMPLES_WATCHDOG_TIMEOUT=49

   The IWDG timer has a range of about 35 seconds and should not be an issue.

usbnsh:
-------

This is another NSH example.  If differs from other 'nsh' configurations
in that this configurations uses a USB serial device for console I/O.
Such a configuration is useful on the stm32f3discovery which has no
builtin RS-232 drivers.

Status:  As of this writing, this configuration has not ran properly.
There appears to be some kind of driver-related issue.

NOTES:

1. This configuration uses the mconf-based configuration tool.  To
   change this configuration using that tool, you should:

   a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
      see additional README.txt files in the NuttX tools repository.

   b. Execute 'make menuconfig' in nuttx/ in order to start the
      reconfiguration process.

2. By default, this configuration uses the ARM EABI toolchain
   for Windows and builds under Cygwin (or probably MSYS).  That
   can easily be reconfigured, of course.

   Build Setup::
         CONFIG_HOST_WINDOWS=y                   : Builds under Windows
         CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin

   System Type::
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y      : GNU EABI toolchain for Windows

3. This configuration does have USART2 output enabled and set up as
       the system logging device:

       Device Drivers -> System Logging Device Options::
         CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
         CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : USART2 will be /dev/ttyS0

       However, there is nothing to generate SYSLOG output in the default
       configuration so nothing should appear on USART2 unless you enable
       some debug output or enable the USB monitor.

       NOTE:  Using the SYSLOG to get debug output has limitations.  Among
       those are that you cannot get debug output from interrupt handlers.
       So, in particularly, debug output is not a useful way to debug the
       USB device controller driver.  Instead, use the USB monitor with
       USB debug off and USB trance on (see below).

4. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system logging device (USART2 in this
       configuration):

        Device Drivers -> "USB Device Driver Support::
          CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
          CONFIG_USBDEV_TRACE_NRECORDS=256        : Buffer 128 records in memory

        Application Configuration -> NSH LIbrary::
          CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
          CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor

        Application Configuration -> System NSH Add-Ons::
          CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
          CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
          CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
          CONFIG_USBMONITOR_INTERVAL=1     : Dump trace data every second
          CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
          CONFIG_USBMONITOR_TRACECLASS=y
          CONFIG_USBMONITOR_TRACETRANSFERS=y
          CONFIG_USBMONITOR_TRACECONTROLLER=y
          CONFIG_USBMONITOR_TRACEINTERRUPTS=y

       NOTE: USB debug output also be enabled in this case.  Both will appear
       on the serial SYSLOG output.  However, the debug output will be
       asynchronous with the trace output and, hence, difficult to interpret.

5. The STM32F3Discovery board does not provide circuitry for control of
   the "soft connect" USB pullup.  As a result, the host PC does not know
   the USB has been logically connected or disconnected.  You have to
   follow these steps to use USB:

   1) Start NSH with USB disconnected, then
   2) Connect the USB device to the host.

6. Using the Prolifics PL2303 Emulation

   You could also use the non-standard PL2303 serial device instead of
   the standard CDC/ACM serial device by changing::

     Drivers->USB Device Driver Support
         CONFIG_CDCACM=n               : Disable the CDC/ACM serial device class
         CONFIG_CDCACM_CONSOLE=n       : The CDC/ACM serial device is NOT the console
         CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
         CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console
