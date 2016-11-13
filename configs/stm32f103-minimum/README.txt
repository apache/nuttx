README
======

This README discusses issues unique to NuttX configurations for the
STM32F103C8T6 Minimum System Development Board for ARM Microcontroller.

This board is available from several vendors on the net, and may
be sold under different names or no name at all. It is based on a
STM32F103C8T6 and has a DIP-40 form-factor.

There are two versions of very similar boards:  One is red and one is
blue.  See http://www.stm32duino.com/viewtopic.php?f=28&t=117

The Red Board:

  Good things about the red board:

  - 1.5k pull up resistor on the PA12 pin (USB D+) which you can
    programatically drag down for automated USB reset.
  - large power capacitors and LDO power.

  Problems with the red board:

  - Silk screen is barely readable, the text is chopped off on some of
    the pins
  - USB connector only has two anchor points and it is directly soldered
    on the surface
  - Small reset button with hardly any resistance

The Blue Board:

  Good things about the blue board:

  - Four soldered anchor point on the USB connector. What you can't tell
    from this picture is that there is a notch in the pcb board and the USB
    connector sits down inside it some. This provides some lateral stability
    that takes some of the stress off the solder points.
  - It has nice clear readable silkscreen printing.
  - It also a larger reset button.

  Problems with the blue board:

  - Probably won't work as a USB device if it has a 10k pull-up on PA12. You
    have to check the pull up on PA12 (USB D+). If it has a 10k pull-up
    resistor, you will need to replace it with a 1.5k one to use the native
    USB.
  - Puny voltage regulator probably 100mA.

  A schematic for the blue board is available here:
  http://www.stm32duino.com/download/file.php?id=276

Both Boards:

  Nice features common to both:

  - SWD pins broken out and easily connected (VCC, GND, SWDIO, SWCLK)
  - USB 5V is broken out with easy access.
  - User LED on PC13
  - Power LED
  - You can probably use more flash (128k) than officially documented for
    the chip (stm32f103c8t6 64k), I was able to load 115k of flash on mine
    and it seemed to work.

  Problems with both boards:

  - No preloaded bootloader * to me this isn't really a problem as the
    entire 64k of flash is available for use
  - No user button

This is the board pinout based on its form-factor for the Blue board:

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
  - Using 128KiB of Flash instead of 64KiB
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

Using 128KiB of Flash instead of 64KiB
======================================

Some people figured out that the STM32F103C8T6 has 128KiB of internal memory
instead of 64KiB as documented in the datasheet and reported by its internal
register.

In order to enable 128KiB you need modify the linker script to reflect this
new size. Open the configs/stm32f103-minimum/scripts/ld.script and replace:

  flash (rx) : ORIGIN = 0x08000000, LENGTH = 64K

with

  flash (rx) : ORIGIN = 0x08000000, LENGTH = 128K

Enable many NuttX features (ie. many filesystems and applications) to get a
large binary image with more than 64K.

We will use OpenOCD to write the firmware in the STM32F103C8T6 Flash. Use a
up to dated OpenOCD version (ie. openocd-0.9).

You will need to create a copy of original openocd/scripts/target/stm32f1x.cfg
to openocd/scripts/target/stm32f103c8t6.cfg and edit the later file replacing:

  flash bank $_FLASHNAME stm32f1x 0x08000000 0 0 0 $_TARGETNAME

with

  flash bank $_FLASHNAME stm32f1x 0x08000000 0x20000 0 0 $_TARGETNAME

We will use OpenOCD with STLink-V2 programmer, but it will work with other
programmers (JLink, Versaloon, or some based on FTDI FT232, etc).

Open a terminal and execute:

  $ sudo openocd -f interface/stlink-v2.cfg -f target/stm32f103c8t6.cfg

Now in other terminal execute:

  $ telnet localhost 4444

  Trying 127.0.0.1...
  Connected to localhost.
  Escape character is '^]'.
  Open On-Chip Debugger

  > reset halt
  stm32f1x.cpu: target state: halted
  target halted due to debug-request, current mode: Thread
  xPSR: 0x01000000 pc: 0x080003ac msp: 0x20000d78

  > flash write_image erase nuttx.bin 0x08000000
  auto erase enabled
  device id = 0x20036410
  ignoring flash probed value, using configured bank size
  flash size = 128kbytes
  stm32f1x.cpu: target state: halted
  target halted due to breakpoint, current mode: Thread
  xPSR: 0x61000000 pc: 0x2000003a msp: 0x20000d78
  wrote 92160 bytes from file nuttx.bin in 4.942194s (18.211 KiB/s)

  > reset run
  > exit

Now NuttX should start normally.

STM32F103 Minimum - specific Configuration Options
==================================================

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
    CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
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

  minnsh:
  ------

    This is a experiment to see just how small we can get a usable NSH
    configuration.  This configuration has far fewer features than the nsh
    configuration but is also a fraction of the size.

    This minnsh configuration is a "proof-of-concept" and not very usable in
    its current state.  This configuration was created by disabling
    everything possible INCLUDING file system support.  Without file system
    support, NuttX is pretty much crippled.  Here are some of the
    consequences of disabling the file system:

    - All features that depend on the file system are lost:  device drivers,
      mountpoints, message queues, named semaphores.

    - Without device drivers, you cannot interact with the RTOS using POSIX
      interfaces.  You would have to work with NuttX as with those other
      tiny RTOSs:  As a scheduler and a callable hardare abstraction layer
      (HAL).

    - You cannot use any of the NuttX upper half device drivers since they
      depend on the pseudo-file system and device nodes.  You can, of
      course, continue to use the lower half drivers either directly.  Or,
      perhaps, you could write some custom minnsh upper half drivers that
      do not depend on a file system and expose a HAL interface.

    There is a special version of readline() the NSH uses when there is no
    file system.  It uses a special up_putc() to write data to the console
    and a special function up_getc() to read data from the console.

    - The current up_getc() implementationsa are a kludge.  They are
      analogous to the up_putc() implementations:  They directly poll the
      hardware for serial availability, locking up all lower priority tasks
      in the entire system while they poll.  So a version of NSH that uses
      up_getc() essentially blocks the system until a character is received.

    This, of course, could be fixed by creating a special, upper half
    implementation of the interrupt-driven serial lower half (like
    stm32_serial) that just supports single character console I/O
    (perhaps called up_putc and up_getc?).  The NSH could wait for serial
    input without blocking the system.  But then that would increase the
    footprint too.

    So although the minnsh configurations are a good starting point for
    making things small, they not are really very practical.  Why might
    you want a NuttX minnsh solution?  Perhaps you have software that runs
    on a family of chips including some very tiny MCUs.  Then perhaps having
    the RTOS compatibility would justify the loss of functionality?

    STATUS:
    2016-06-03:  Using that config I got this:

        $ ls -l nuttx.bin
        -rwxr-xr-x 1 alan alan 12543 Jun 3 17:58 nuttx.bin

        $ arm-none-eabi-size nuttx
        text data bss dec hex filename
        12542 1 816 13359 342f nuttx

      And this is free command from NuttX shell:

        NuttShell (NSH)
        nsh> free
        total used free largest
        Mem: 18624 2328 16296 16296
       nsh>

    2016-06-07:  As another experiment, I tried enabling just (1) the file
    system, (2) the console device, and (3) the upper half serial driver in
    the minnsh configuration.  With these changes, NSH should behave better 
    and we preserve the device driver interface.  I made the following
    configuration changes:

      Enable the file system:
        CONFIG_NFILE_DESCRIPTORS=5
        CONFIG_NFILE_STREAMS=5

      Enable the console device:
        CONFIG_DEV_CONSOLE=y

      Disable most new NSH commands.  Some like 'ls' are really mandatory
      with a file system:
        CONFIG_NSH_DISABLE_xxx=y

      Enable the upper half serial driver:
        CONFIG_SERIAL=y
        CONFIG_STANDARD_SERIAL=y

      Enable the USART1 serial driver:
        CONFIG_STM32_USART1=y
        CONFIG_STM32_USART1_SERIALDRIVER=y
        CONFIG_USART1_SERIAL_CONSOLE=y

        CONFIG_USART1_2STOP=0
        CONFIG_USART1_BAUD=115200
        CONFIG_USART1_BITS=8
        CONFIG_USART1_PARITY=0
        CONFIG_USART1_RXBUFSIZE=16
        CONFIG_USART1_TXBUFSIZE=16

    The resulting code was bigger as expected:

      $ arm-none-eabi-size nuttx
       text    data     bss     dec     hex filename
      19853      88     876   20817    5151 nuttx

    I am sure that other things that could be disabled were also drawn into
    the build, so perhaps this could be reduced.  This amounts to a size
    increase of around 7KB.

    One major part of this size increase is due to the addition of the NSH
    'ls' command.  Now, if I disable the 'ls' command, I get:

      $ arm-none-eabi-size nuttx
       text    data     bss     dec     hex filename
      17804      80     864   18748    493c nuttx

    Or an increase of only 5.1 KB.  This, of course, not only excludes the
    'ls' command logic, but also the things that were drawn into the link
    when 'ls' was enabled:  opendir(), readdir(), closedir(), stat(), and
    probably other things.

    So I think we can say that the cost of the file system and true serial
    console device was about 5 KB (primarily OS support) and the cost of
    the NSH 'ls' command (including OS support) is about 2KB.

    2016-06-21:  Just checking the size after some big system changes:  The
    size of the base configuration has actually dropped by a few bytes:

     $ arm-none-eabi-size nuttx
        text    data     bss     dec     hex filename
      12526       4     816   13346    3422 nuttx

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a console on UART1. Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

  jlx12864g:
  ---------
    This is a config example to use the JLX12864G-086 LCD module. To use this
    LCD you need to connect PA5 (SPI1 CLK) to SCK; PA7 (SPI1 MOSI) to SDA; PA4
    to CS; PA3 to RST; PA2 to RS.

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
       CONFIG_USBMONITOR=y              : Enable the USB monitor daemon
       CONFIG_USBMONITOR_STACKSIZE=2048 : USB monitor daemon stack size
       CONFIG_USBMONITOR_PRIORITY=50    : USB monitor daemon priority
       CONFIG_USBMONITOR_INTERVAL=2     : Dump trace data every 2 seconds

       CONFIG_USBMONITOR_TRACEINIT=y    : Enable TRACE output
       CONFIG_USBMONITOR_TRACECLASS=y
       CONFIG_USBMONITOR_TRACETRANSFERS=y
       CONFIG_USBMONITOR_TRACECONTROLLER=y
       CONFIG_USBMONITOR_TRACEINTERRUPTS=y

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

  veml6070:
  --------
    This is a config example to use the Vishay VEML6070 UV-A sensor. To use this
    sensor you need to connect PB6 (I2C1 CLK) to SCL; PB7 (I2C1 SDA) to SDA of
    sensor module. I used a GY-VEML6070 module to test this driver.
