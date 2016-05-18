README
======

This README discusses issues unique to NuttX configurations for the
STM32F103C8T6 Minimum System Development Board for ARM Microcontroller.

This board is available from several vendors on the net, and may
be sold under different names or no name at all. It is based on a
STM32F103C8T6 and has a DIP-40 form-factor.

This is the board pinout based on its form-factor:

      USB
      ___
-----/ _ \-----
|B12       GND|
|B13       GND|
|B14      3.3V|
|B15       RST|
|A8        B11|
|A9        B10|
|A10        B1|
|A11        B0|
|A12        A7|
|A15        A6|
|B3         A5|
|B4         A4|
|B5         A3|
|B6         A2|
|B7         A1|
|B8         A0|
|B9        C15|
|5V        C14|
|GND       C13|
|3.3V       VB|
|_____________|

Contents
========

  - LEDs
  - UARTs
  - Timer Inputs/Outputs
  - STM32F103 Minimum - specific Configuration Options
  - Configurations

LEDs
====

The STM32F103 Minimum board has only one software controllable LED.
This LED can be used by the board port when CONFIG_ARCH_LEDS option is
enabled.

If enabled the LED is simply turned on when the board boots
succesfully, and is blinking on panic / assertion failed.

UARTs
=====

UART/USART PINS
---------------

USART1
  RX      PA10
  TX      PA9
USART2
  CK      PA4
  CTS     PA0
  RTS     PA1
  RX      PA3
  TX      PA2
USART3
  CK      PB12
  CTS     PB13
  RTS     PB14
  RX      PB11
  TX      PB10

Default USART/UART Configuration
--------------------------------

USART1 (RX & TX only) is available through pins PA9 (TX) and PA10 (RX).

Timer Inputs/Outputs
====================

TIM1
  CH1     PA8
  CH2     PA9*
  CH3     PA10*
  CH4     PA11*
TIM2
  CH1     PA0*, PA15, PA5
  CH2     PA1, PB3
  CH3     PA2, PB10*
  CH4     PA3, PB11
TIM3
  CH1     PA6, PB4
  CH2     PA7, PB5*
  CH3     PB0
  CH4     PB1*
TIM4
  CH1     PB6*
  CH2     PB7
  CH3     PB8
  CH4     PB9*

 * Indicates pins that have other on-board functions and should be used only
   with care (See board datasheet).

STM32F103 Minimum - specific Configuration Options
===============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F103C8=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm32f103-minium

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32_MINIMUM=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=20480 (20Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    AHB
    ---
    CONFIG_STM32_CRC
    CONFIG_STM32_BKPSRAM

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_CAN1
    CONFIG_STM32_PWR -- Required for RTC

    APB2
    ----
    CONFIG_STM32_TIM1
    CONFIG_STM32_USART1
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2
    CONFIG_STM32_SPI1

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation or ADC conversion.
  Note that ADC require two definitions:  Not only do you have
  to assign the timer (n) for used by the ADC, but then you also have to
  configure which ADC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  JTAG Enable settings (by default only SW-DP is enabled):

    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32F103 Minimum specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3)
       for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  STM32F103 Minimum CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
      CONFIG_STM32_CAN2 must also be defined)
    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
      Default: 8
    CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
      Default: 4
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2 is defined.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7
    CONFIG_CAN_REGDEBUG - If CONFIG_DEBUG is set, this will generate an
      dump of all CAN registers.

  STM32F103 Minimum SPI Configuration

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

Configurations
==============

Each STM32F103 Minimum configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh STM32F103 Minimum/<subdir>
    cd -
    . ./setenv.sh

If this is a Windows native build, then configure.bat should be used
instead of configure.sh:

    configure.bat STM32F103-Minimum\<subdir>

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a console on UART1. Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

  usbnsh:
  -------

    This is another NSH example.  If differs from other 'nsh' configurations
    in that this configurations uses a USB serial device for console I/O.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_WINDOWS=y                   : Builds under Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery for Windows

    3. This configuration does have UART2 output enabled and set up as
       the system logging device:

       CONFIG_SYSLOG=y                    : Enable output to syslog, not console
       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART2 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on UART2 unless you enable
       some debug output or enable the USB monitor.

    4. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system loggin device (UART2 in this
       configuraion):

       CONFIG_USBDEV_TRACE=y                   : Enable USB trace feature
       CONFIG_USBDEV_TRACE_NRECORDS=128        : Buffer 128 records in memory
       CONFIG_NSH_USBDEV_TRACE=n               : No builtin tracing from NSH
       CONFIG_NSH_ARCHINIT=y                   : Automatically start the USB monitor
       CONFIG_SYSTEM_USBMONITOR=y              : Enable the USB monitor daemon
       CONFIG_SYSTEM_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
       CONFIG_SYSTEM_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
       CONFIG_SYSTEM_USBMONITOR_INTERVAL=2     : Dump trace data every 2 seconds

       CONFIG_SYSTEM_USBMONITOR_TRACEINIT=y    : Enable TRACE output
       CONFIG_SYSTEM_USBMONITOR_TRACECLASS=y
       CONFIG_SYSTEM_USBMONITOR_TRACETRANSFERS=y
       CONFIG_SYSTEM_USBMONITOR_TRACECONTROLLER=y
       CONFIG_SYSTEM_USBMONITOR_TRACEINTERRUPTS=y

    5. By default, this project assumes that you are *NOT* using the DFU
       bootloader.

    Using the Prolifics PL2303 Emulation
    ------------------------------------
    You could also use the non-standard PL2303 serial device instead of
    the standard CDC/ACM serial device by changing:

      CONFIG_CDCACM=y               : Disable the CDC/ACM serial device class
      CONFIG_CDCACM_CONSOLE=y       : The CDC/ACM serial device is NOT the console
      CONFIG_PL2303=y               : The Prolifics PL2303 emulation is enabled
      CONFIG_PL2303_CONSOLE=y       : The PL2303 serial device is the console
