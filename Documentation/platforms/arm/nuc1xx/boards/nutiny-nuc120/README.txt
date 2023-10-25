README.txt
==========

This is the README file for the port of NuttX to the NuvoTon
NuTiny-SDK-NUC120 board.  This board has the NUC120LE3AN chip
with a built-in NuLink debugger.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - NuttX Buildroot Toolchain
  - LEDs
  - Serial Console
  - Debugging
  - NuTiny-specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
=====================

  As of this writing, all testing has been performed using the NuttX buildroot
  toolchain described below.  I have also verified the build using the
  CodeSourcery GCC toolchain for windows.  Most any contemporary EABI GCC
  toolchain should work will a little tinkering.

NuttX Buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The PATH environment variable should
  be modified to point to the correct path to the Cortex-M0 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M0 toolchain, one can be downloaded from the NuttX
  Bitbucket download site (https://bitbucket.org/nuttx/buildroot/downloads/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured NuttX in <some-dir>/nuttx.

     tools/configure.sh nutiny-nuc120:<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp boards/cortexm0-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Make sure that the PATH variable includes the path to the newly built
     binaries.

  See the file boards/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M0 toolchain for Cygwin under Windows.

LEDs
====

  The NuTiny has a single green LED that can be controlled from software.
  This LED is connected to PIN17.  It is pulled high so a low value will
  illuminate the LED.

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
  NuTiny.  The following definitions describe how NuttX controls the LEDs:

    SYMBOL                Meaning                 LED state
                                                  Initially all LED is OFF
    -------------------  -----------------------  ------------- ------------
    LED_STARTED          NuttX has been started   LED ON
    LED_HEAPALLOCATE     Heap has been allocated  LED ON
    LED_IRQSENABLED      Interrupts enabled       LED ON
    LED_STACKCREATED     Idle stack created       LED ON
    LED_INIRQ            In an interrupt          LED should glow
    LED_SIGNAL           In a signal handler      LED might glow
    LED_ASSERTION        An assertion failed      LED ON while handling the assertion
    LED_PANIC            The system has crashed   LED Blinking at 2Hz
    LED_IDLE             NUC1XX is in sleep mode  (Optional, not used)

Serial Console
==============

As with most NuttX configurations, the NuTiny-SKD-NUC120 configurations
depend on having a serial console to interact with the software.  The
NuTiny-SDK-NUC120, however, has not on-board RS-232 drivers so will be
necessary to connect the NuTiny-SDK-NUC120 UART pins to an external
RS-232 driver board or TTL-to-Serial USB adaptor.

By default UART1 is used as the serial console on these boards.  NUC120LE3AN
is provided as an LQFP48 package and, for this case, the UART1 RX signal
(RXD1) is on PB.4, pin 8, and the TX signal (TXD1) is on PB.5, pin 9.
These pins are available on the NuTiny-SDC-NUC120 JP5.

  NOTE: The TX vs RX labeling may be confusing.  On one RS-232 driver board,
  I had to connect the NUC120 TXD0 pin to the driver boards RXD pin.  How
  confusing!

UART0 is an alternative that can be selected by modifying the default
configuration.  UART0 RX (RXD0) is on PB.0, pin 17, and the TX signal (TXD0)
is on PB.1, pin 18.  These pins are available on the NuTiny-SDC-NUC120 JP1.

  NOTE: PB.0, pin 17, is also used to control the user LED on board (labeled
  "IO").  CONFIG_ARCH_LED should not be selected if UART0 is used.

The NUC120LE3AN does not support UART2.

Debugging
=========

The NuTiny-SDK-NUC120 includes a built-in NuLink debugger.  Unfortunately,
full debug support is available only with the Keil and IAR toolchains.
There is, however, a free program called ICP (In-Circuit Programmer).  It
can be used to burn programs into FLASH (aka APROM).

The ICP program can also be used to burn an ISP program into LDROM.  The
ISP (In-System Programmer) is available free from the Nuvton website.

Then NuttX build does not set the configuration words at 0x0030000-0x00300004.
You should uncheck the Config box when burning APROM or the previous contents
of the configuration words will be erased.

NuTiny-specific Configuration Options
=====================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM0=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=nuc1xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_NUC120LE3AN=y

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=nutiny-nuc120 (for the NuTiny-SDK-NUC120 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_NUTINY_NUC120=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=16384 (16Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  Individual subsystems can be enabled as follows.  These settings are for
  all of the NUC100/120 line and may not be available for the NUC120LE3AN
  in particular:

  AHB
  ---

    CONFIG_NUC_PDMA    Peripheral DMA
    CONFIG_NUC_FMC     Flash memory
    CONFIG_NUC_EBI     External bus interface

  APB1
  ----

    CONFIG_NUC_WDT     Watchdog timer
    CONFIG_NUC_RTC     Real time clock (RTC)
    CONFIG_NUC_TMR0    Timer0
    CONFIG_NUC_TMR1    Timer1
    CONFIG_NUC_I2C0    I2C interface
    CONFIG_NUC_SPI0    SPI0 master/slave
    CONFIG_NUC_SPI1    SPI1 master/slave
    CONFIG_NUC_PWM0    PWM0
    CONFIG_NUC_PWM1    PWM1
    CONFIG_NUC_PWM2    PWM2
    CONFIG_NUC_PWM3    PWM3
    CONFIG_NUC_UART0   UART0
    CONFIG_NUC_USBD    USB 2.0 FS device controller
    CONFIG_NUC_ACMP    Analog comparator
    CONFIG_NUC_ADC     Analog-digital-converter (ADC)

  APB2
  ---

    CONFIG_NUC_PS2     PS/2 interface
    CONFIG_NUC_TIMR2   Timer2
    CONFIG_NUC_TIMR3   Timer3
    CONFIG_NUC_I2C1    I2C1 interface
    CONFIG_NUC_SPI2    SPI2 master/slave
    CONFIG_NUC_SPI3    SPI3 master/slave
    CONFIG_NUC_PWM4    PWM4
    CONFIG_NUC_PWM5    PWM5
    CONFIG_NUC_PWM6    PWM6
    CONFIG_NUC_PWM7    PWM7
    CONFIG_NUC_UART1   UART1
    CONFIG_NUC_UART2   UART2
    CONFIG_NUC_I2S     I2S interface

  NUC1XX specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - Selects the UARTn (n=0,1,2) for the
      console and ttys0.
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer for UARTn.
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
       for UARTn.
    CONFIG_UARTn_BAUD - The configure BAUD of UARTn,
    CONFIG_UARTn_BITS - The number of bits.  Must be 5, 6, 7, or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

Configurations
==============

Each NuTiny-SDK-NUC120 configuration is maintained in a sub-directory and
can be selected as follow:

    tools/configure.sh nutiny-nuc120:<subdir>

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART1.  Support for
    builtin applications is disabled.

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

    3. Serial Console.  A serial console is necessary to interrupt with
       NSH. The serial console is configured on UART1 which is available
       on JP5:

       UART1 RX signal (RXD1) is on PB.4, pin 8, and
       UART1 TX signal (TXD1) is on PB.5, pin 9.

       It is possible to configure NSH to use a USB serial console instead
       of an RS-232 serial console.  However, that configuration has not
       been impelmented as of this writing.

    4. Memory Usage.  The size command gives us the static memory usage.
       This is what I get:

       $ size nuttx
          text    data     bss     dec     hex filename
         35037     106    1092   36235    8d8b nuttx

       And we can get the runtime memory usage from the NSH free command:

       NuttShell (NSH) NuttX-6.25
       nsh> free
            total  used free  largest
       Mem: 14160  3944 10216 10216
       nsh>

       Summary:

       - This slightly tuned NSH example uses 34.2KB of FLASH leaving 93.8KB
         of FLASH (72%) free from additional application development.

         I did not do all of the arithmetic, but it appears to me that of this
         34+KB of FLASH usage, probably 20-30% of the FLASH is used by libgcc!
         libgcc has gotten very fat!

       - Static SRAM usage is about 1.2KB (<4%).

       - At run time, 10.0KB of SRAM (62%) is still available for additional
         applications. Most of the memory used at runtime is allocated I/O
         buffers and the stack for the NSH main thread (1.5KB).

       There is probably enough free memory to support 3 or 4 application
       threads in addition to NSH.
