README.txt
==========

This is the README file for the port of NuttX to the NuvoTon
NuTiny-SDK-NUC120 board.  This board has the NUC120LE3AN chip
with a built-in NuLink debugger.

Contents
========

  - Development Environment
  - GNU Toolchain Options
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

  As of this writing I have used only the CodeSourcery GCC toolchain for windows.

LEDs
====

  The NuTiny has a single green LED that can be controlled from sofware.
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
    LED_IDLE             NUC1XX is is sleep mode  (Optional, not used)

Serial Console
==============

To be provided

Debugging
=========

The NuTiny-SDK-NUC120 includes a built-in NuLink debugger.  Unfortunately,
full debug support is available only with the Keil and IAR toolchains.
There is, however, a free program call call ICP (In-Circuit Programmer).
It can be used to burn programs into FLASH (aka APROM).

The ICP program can also be used to burn an ISP program into LDROM.  The
ISP (In-System Programmer) is available free from the Nuvton website.

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

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=nutiny-nuc120 (for the NuTiny-SDK-NUC120 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_NUTINY_NUC120=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=16384 (16Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The Cortex-M0 supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
       stack in bytes.  If not defined, the user task stacks will be
       used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  Individual subsystems can be enabled:

  AHB
  ---

    CONFIG_NUC_PDMA    Peripheral DMA
    CONFIG_NUC_FMC     Flash memory
    CONFIG_NUC_EBI     External bus interface

  APB1
  ----

    CONFIG_NUC_WDT     Watchdog timer
    CONFIG_NUC_RTC     Real time clock (RTC)
    CONFIG_NUC_TMR01   Timer0/Timer1
    CONFIG_NUC_I2C0    I2C interface
    CONFIG_NUC_SPI0    SPI0 master/slave
    CONFIG_NUC_SPI1    SPI1 master/slave
    CONFIG_NUC_PWMA    PWM0/1/2/3
    CONFIG_NUC_UART0   UART0
    CONFIG_NUC_USBD    USB 2.0 FS device controller registers */
    CONFIG_NUC_ACMP    Analog comparator
    CONFIG_NUC_ADC     Analog-digital-converter (ADC)

  APB2
  ---

    CONFIG_NUC_PS2     PS/2 interface
    CONFIG_NUC_TIMR23  Timer2/Timer3
    CONFIG_NUC_I2C1    I2C1 interface
    CONFIG_NUC_SPI2    SPI2 master/slave
    CONFIG_NUC_SPI3    SPI3 master/slave
    CONFIG_NUC_PWMB    PWM4/5/6/7
    CONFIG_NUC_UART1   UART1
    CONFIG_NUC_UART2   UART2
    CONFIG_NUC_I2S     I2S interface

  NUC1XX specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn (n=0,1,2) for the
      console and ttys0 (default is the USART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be 5, 6, 7, or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

Configurations
==============

Each NuTiny-SDK-NUC120 configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh nutiny-nuc120/<subdir>
    cd -
    . ./setenv.sh

If this is a Windows native build, then configure.bat should be used
instead of configure.sh:

    configure.bat nutiny-nuc120\<subdir>

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default toolchain:

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV6M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART0.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected (see NOTES below).

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration includes USB Support (CDC/ACM device)

       CONFIG_STM32_USB=y            : STM32 USB device support
       CONFIG_USBDEV=y               : USB device support must be enabled
       CONFIG_CDCACM=y               : The CDC/ACM driver must be built
       CONFIG_NSH_BUILTIN_APPS=y     : NSH built-in application support must be enabled
       CONFIG_NSH_ARCHINIT=y         : To perform USB initialization

       The CDC/ACM example is included as two NSH "built-in" commands.\
 
       CONFIG_EXAMPLES_CDCACM=y      : Enable apps/examples/cdcacm
  
       The two commands are:
 
       sercon : Connect the serial device a create /dev/ttyACM0
       serdis : Disconnect the serial device.        

       NOTE:  The serial connections/disconnections do not work as advertised.
       This is because the NuTiny-SDK-NUC120 board does not provide circuitry for
       control of the "soft connect" USB pullup.  As a result, the host PC
       does not know the USB has been logically connected or disconnected.  You
       have to follow these steps to use USB:

       1) Start NSH with USB disconnected
       2) enter to 'sercon' command to start the CDC/ACM device, then
       3) Connect the USB device to the host.
 
       and to close the connection:

       4) Disconnect the USB device from the host
       5) Enter the 'serdis' command
