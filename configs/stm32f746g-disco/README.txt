README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM32F746G-DISCO development board featuring the STM32F746NGH6 
MCU. The STM32F746NGH6  is a 216MHz Cortex-M7 operation with 1024Kb Flash
memory and 300Kb SRAM. The board features:

  - On-board ST-LINK/V2 for programming and debugging,
  - Mbed-enabled (mbed.org)
  - 4.3-inch 480x272 color LCD-TFT with capacitive touch screen
  - Camera connector
  - SAI audio codec
  - Audio line in and line out jack
  - Stereo speaker outputs
  - Two ST MEMS microphones
  - SPDIF RCA input connector
  - Two pushbuttons (user and reset)
  - 128-Mbit Quad-SPI Flash memory
  - 128-Mbit SDRAM (64 Mbits accessible)
  - Connector for microSD card
  - RF-EEPROM daughterboard connector
  - USB OTG HS with Micro-AB connectors
  - USB OTG FS with Micro-AB connectors
  - Ethernet connector compliant with IEEE-802.3-2002

Refer to the http://www.st.com website for further information about this
board (search keyword: stm32f746g-disco)

Contents
========

  - Development Environment
  - LEDs and Buttons
  - Serial Console
  - FPU
  - STM32F746G-DISCO-specific Configuration Options
  - Configurations

Development Environment
=======================

  The Development environments for the STM32F746G-DISCO board are identical
  to the environments for other STM32F boards.  For full details on the
  environment options and setup, see the README.txt file in the
  config/stm32f746g-disco directory.

LEDs and Buttons
================

  LEDs
  ----
  The STM32F746G-DISCO board has numerous LEDs but only one, LD1 located
  near the reset button, that can be controlled by software (LD2 is a power
  indicator, LD3-6 indicate USB status, LD7 is controlled by the ST-Link).

  LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino
  interface.  One end of LD1 is grounded so a high output on PI1 will
  illuminate the LED.

  This LED is not used by the board port unless CONFIG_ARCH_LEDS is defined.
  In that case, the usage by the board port is defined in include/board.h
  and src/stm32_leds.c. The LEDs are used to encode OS-related events as
  follows:

    SYMBOL              Meaning                 LD1
    ------------------- ----------------------- ------
    LED_STARTED         NuttX has been started  OFF
    LED_HEAPALLOCATE    Heap has been allocated OFF
    LED_IRQSENABLED     Interrupts enabled      OFF
    LED_STACKCREATED    Idle stack created      ON
    LED_INIRQ           In an interrupt         N/C
    LED_SIGNAL          In a signal handler     N/C
    LED_ASSERTION       An assertion failed     N/C
    LED_PANIC           The system has crashed  FLASH

  Thus is LD1 is statically on, NuttX has successfully  booted and is,
  apparently, running normally.  If LD1 is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.

  Buttons
  -------
  Pushbutton B1, labelled "User", is connected to GPIO PI11.  A high
  value will be sensed when the button is depressed.

Serial Console
==============

  These configurations assume that you are using a standard Arduio RS-232
  shield with the serial interface with RX on pin D0 and TX on pin D1:

  -------- ---------------
              STM32F7
  ARDUIONO FUNCTION  GPIO
  -- ----- --------- -----
  DO RX    USART6_RX PC7
  D1 TX    USART6_TX PC6
  -- ----- --------- -----

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the STM32 port.

1. Lazy Floating Point Register Save.

   This is an implementation that saves and restores FPU registers only on
   context switches.  This means: (1) floating point registers are not
   stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file:

   CONFIG_ARCH_FPU=y
   CONFIG_ARMV7M_CMNVECTOR=y
   CONFIG_ARMV7M_LAZYFPU=y

2. Non-Lazy Floating Point Register Save

   Mike Smith has contributed an extensive re-write of the ARMv7-M exception
   handling logic. This includes verified support for the FPU.  These changes
   have not yet been incorporated into the mainline and are still considered
   experimental.  These FPU logic can be enabled with:

   CONFIG_ARCH_FPU=y
   CONFIG_ARMV7M_CMNVECTOR=y

   You will probably also changes to the ld.script in if this option is selected.
   This should work:

   -ENTRY(_stext)
   +ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
   +EXTERN(_vectors)       /* Force the vectors to be included in the output */

STM32F746G-DISCO-specific Configuration Options
===============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F746=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=STM32F746G-DISCO (for the STM32F746G-DISCO development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32F746G_DISCO=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x20000000

    In order to use FSMC SRAM, the following additional things need to be
    present in the NuttX configuration file:

    CONFIG_STM32F7_FSMC_SRAM - Indicates that SRAM is available via the
      FSMC (as opposed to an LCD or FLASH).

    CONFIG_HEAP2_BASE - The base address of the SRAM in the FSMC address space (hex)

    CONFIG_HEAP2_SIZE - The size of the SRAM in the FSMC address space (decimal)

    CONFIG_ARCH_FPU - The STM32F746G-DISCO supports a floating point unit (FPU)

       CONFIG_ARCH_FPU=y

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

    APB1
    ----
    CONFIG_STM32F7_TIM2           TIM2
    CONFIG_STM32F7_TIM3           TIM3
    CONFIG_STM32F7_TIM4           TIM4
    CONFIG_STM32F7_TIM5           TIM5
    CONFIG_STM32F7_TIM6           TIM6
    CONFIG_STM32F7_TIM7           TIM7
    CONFIG_STM32F7_TIM12          TIM12
    CONFIG_STM32F7_TIM13          TIM13
    CONFIG_STM32F7_TIM14          TIM14
    CONFIG_STM32F7_LPTIM1         LPTIM1
    CONFIG_STM32F7_RTC            RTC
    CONFIG_STM32F7_BKP            BKP Registers
    CONFIG_STM32F7_WWDG           WWDG
    CONFIG_STM32F7_IWDG           IWDG
    CONFIG_STM32F7_SPI2           SPI2
    CONFIG_STM32F7_I2S2           I2S2
    CONFIG_STM32F7_SPI3           SPI3
    CONFIG_STM32F7_I2S3           I2S3
    CONFIG_STM32F7_SPDIFRX        SPDIFRX
    CONFIG_STM32F7_USART2         USART2
    CONFIG_STM32F7_USART3         USART3
    CONFIG_STM32F7_UART4          UART4
    CONFIG_STM32F7_UART5          UART5
    CONFIG_STM32F7_I2C1           I2C1
    CONFIG_STM32F7_I2C2           I2C2
    CONFIG_STM32F7_I2C3           I2C3
    CONFIG_STM32F7_I2C4           I2C4
    CONFIG_STM32F7_CAN1           CAN1
    CONFIG_STM32F7_CAN2           CAN2
    CONFIG_STM32F7_HDMICEC        HDMI-CEC
    CONFIG_STM32F7_PWR            PWR
    CONFIG_STM32F7_DAC            DAC
    CONFIG_STM32F7_UART7          UART7
    CONFIG_STM32F7_UART8          UART8

    APB2
    ----
    CONFIG_STM32F7_TIM1           TIM1
    CONFIG_STM32F7_TIM8           TIM8
    CONFIG_STM32F7_USART1         USART1
    CONFIG_STM32F7_USART6         USART6
    CONFIG_STM32F7_ADC            ADC1 - ADC2 - ADC3
    CONFIG_STM32F7_SDMMC1         SDMMC1
    CONFIG_STM32F7_SPI1           SPI1
    CONFIG_STM32F7_SPI4           SPI4
    CONFIG_STM32F7_SYSCFG         SYSCFG
    CONFIG_STM32F7_EXTI           EXTI
    CONFIG_STM32F7_TIM9           TIM9
    CONFIG_STM32F7_TIM10          TIM10
    CONFIG_STM32F7_TIM11          TIM11
    CONFIG_STM32F7_SPI5           SPI5
    CONFIG_STM32F7_SPI6           SPI6
    CONFIG_STM32F7_SAI1           SAI1
    CONFIG_STM32F7_SAI2           SAI2
    CONFIG_STM32F7_LTDC           LCD-TFT

    AHB1
    ----
    CONFIG_STM32F7_CRC            CRC
    CONFIG_STM32F7_BKPSRAM        BKPSRAM
    CONFIG_STM32F7_DMA1           DMA1
    CONFIG_STM32F7_DMA2           DMA2
    CONFIG_STM32F7_ETHMAC         Ethernet MAC
    CONFIG_STM32F7_DMA2D          Chrom-ART (DMA2D)
    CONFIG_STM32F7_USBOTGHS       USB OTG HS

    AHB2
    ----
    CONFIG_STM32F7_USBOTGFS       USB OTG FS
    CONFIG_STM32F7_DCMI           DCMI
    CONFIG_STM32F7_CRYP           CRYP
    CONFIG_STM32F7_HASH           HASH
    CONFIG_STM32F7_RNG            RNG

    AHB3
    ----

    CONFIG_STM32F7_FSMC           FSMC control registers
    CONFIG_STM32F7_QUADSPI        QuadSPI Control

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32F7_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32F7_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32F7_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32F7_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
    CONFIG_STM32F7_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
    CONFIG_STM32F7_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32F7_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  JTAG Enable settings (by default only SW-DP is enabled):

    CONFIG_STM32F7_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32F7_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32F7_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32F746G-DISCO specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

  STM32F746G-DISCO CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32F7_CAN1 or
      CONFIG_STM32F7_CAN2 must also be defined)
    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
      Default: 8
    CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
      Default: 4
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32F7_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32F7_CAN2 is defined.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7
    CONFIG_CAN_REGDEBUG - If CONFIG_DEBUG is set, this will generate an
      dump of all CAN registers.

  STM32F746G-DISCO SPI Configuration

    CONFIG_STM32F7_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32F7_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32F7_SPI_INTERRUPT.

  STM32F746G-DISCO DMA Configuration

    CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32F7_SDIO
      and CONFIG_STM32F7_DMA2.
    CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
    CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority.
      Default:  Medium
    CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
      4-bit transfer mode.

  STM32 USB OTG FS Host Driver Support

  Pre-requisites

   CONFIG_USBDEV          - Enable USB device support
   CONFIG_USBHOST         - Enable USB host support
   CONFIG_STM32F7_OTGFS     - Enable the STM32 USB OTG FS block
   CONFIG_STM32F7_SYSCFG    - Needed
   CONFIG_SCHED_WORKQUEUE - Worker thread support is required

  Options:

   CONFIG_STM32F7_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
     Default 128 (512 bytes)
   CONFIG_STM32F7_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
     in 32-bit words.  Default 96 (384 bytes)
   CONFIG_STM32F7_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
     words.  Default 96 (384 bytes)
   CONFIG_STM32F7_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128
   CONFIG_STM32F7_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
     want to do that?
   CONFIG_STM32F7_USBHOST_REGDEBUG - Enable very low-level register access
     debug.  Depends on CONFIG_DEBUG.
   CONFIG_STM32F7_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
     packets. Depends on CONFIG_DEBUG.

Configurations
==============

Each STM32F746G-DISCO configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh stm32f746g-disco/<subdir>
    cd -
    . ./setenv.sh

If this is a Windows native build, then configure.bat should be used
instead of configure.sh:

    configure.bat STM32F746G-DISCO\<subdir>

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART2.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected (see NOTES below).

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

    3. This example supports the PWM test (apps/examples/pwm) but this must
       be manually enabled by selecting:

       CONFIG_PWM=y              : Enable the generic PWM infrastructure
       CONFIG_STM32F7_TIM4=y       : Enable TIM4
       CONFIG_STM32F7_TIM4_PWM=y   : Use TIM4 to generate PWM output

       See also apps/examples/README.txt

       Special PWM-only debug options:

       CONFIG_DEBUG_PWM

    5. This example supports the Quadrature Encode test (apps/examples/qencoder)
       but this must be manually enabled by selecting:

       CONFIG_EXAMPLES_QENCODER=y : Enable the apps/examples/qencoder
       CONFIG_SENSORS=y           : Enable support for sensors
       CONFIG_QENCODER=y          : Enable the generic Quadrature Encoder infrastructure
       CONFIG_STM32F7_TIM8=y        : Enable TIM8
       CONFIG_STM32F7_TIM2=n        : (Or optionally TIM2)
       CONFIG_STM32F7_TIM8_QE=y     : Use TIM8 as the quadrature encoder
       CONFIG_STM32F7_TIM2_QE=y     : (Or optionally TIM2)

       See also apps/examples/README.txt. Special debug options:

       CONFIG_DEBUG_SENSORS

    6. This example supports the watchdog timer test (apps/examples/watchdog)
       but this must be manually enabled by selecting:

       CONFIG_EXAMPLES_WATCHDOG=y : Enable the apps/examples/watchdog
       CONFIG_WATCHDOG=y          : Enables watchdog timer driver support
       CONFIG_STM32F7_WWDG=y        : Enables the WWDG timer facility, OR
       CONFIG_STM32F7_IWDG=y        : Enables the IWDG timer facility (but not both)

       The WWDG watchdog is driven off the (fast) 42MHz PCLK1 and, as result,
       has a maximum timeout value of 49 milliseconds.  for WWDG watchdog, you
       should also add the fillowing to the configuration file:

       CONFIG_EXAMPLES_WATCHDOG_PINGDELAY=20
       CONFIG_EXAMPLES_WATCHDOG_TIMEOUT=49

       The IWDG timer has a range of about 35 seconds and should not be an issue.

     7. USB Support (CDC/ACM device)

        CONFIG_STM32F7_OTGFS=y          : STM32 OTG FS support
        CONFIG_USBDEV=y               : USB device support must be enabled
        CONFIG_CDCACM=y               : The CDC/ACM driver must be built
        CONFIG_NSH_BUILTIN_APPS=y     : NSH built-in application support must be enabled
        CONFIG_NSH_ARCHINIT=y         : To perform USB initialization

     8. Using the USB console.

        The STM32F746G-DISCO NSH configuration can be set up to use a USB CDC/ACM
        (or PL2303) USB console.  The normal way that you would configure the
        the USB console would be to change the .config file like this:

        CONFIG_STM32F7_OTGFS=y           : STM32 OTG FS support
        CONFIG_USART2_SERIAL_CONSOLE=n : Disable the USART2 console
        CONFIG_DEV_CONSOLE=n           : Inhibit use of /dev/console by other logic
        CONFIG_USBDEV=y                : USB device support must be enabled
        CONFIG_CDCACM=y                : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=y        : Enable the CDC/ACM USB console.

        NOTE: When you first start the USB console, you have hit ENTER a few
        times before NSH starts.  The logic does this to prevent sending USB data
        before there is anything on the host side listening for USB serial input.

    9.  Here is an alternative USB console configuration.  The following
        configuration will also create a NSH USB console but this version
        will use /dev/console.  Instead, it will use the normal /dev/ttyACM0
        USB serial device for the console:

        CONFIG_STM32F7_OTGFS=y           : STM32 OTG FS support
        CONFIG_USART2_SERIAL_CONSOLE=y : Keep the USART2 console
        CONFIG_DEV_CONSOLE=y           : /dev/console exists (but NSH won't use it)
        CONFIG_USBDEV=y                : USB device support must be enabled
        CONFIG_CDCACM=y                : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=n        : Don't use the CDC/ACM USB console.
        CONFIG_NSH_USBCONSOLE=y        : Instead use some other USB device for the console

        The particular USB device that is used is:

        CONFIG_NSH_USBCONDEV="/dev/ttyACM0"

        The advantage of this configuration is only that it is easier to
        bet working.  This alternative does has some side effects:

        - When any other device other than /dev/console is used for a user
          interface, linefeeds (\n) will not be expanded to carriage return /
          linefeeds (\r\n).  You will need to set your terminal program to account
          for this.

        - /dev/console still exists and still refers to the serial port. So
          you can still use certain kinds of debug output (see include/debug.h, all
          of the interfaces based on lowsyslog will work in this configuration).

        - But don't enable USB debug output!  Since USB is console is used for
          USB debug output and you are using a USB console, there will be
          infinite loops and deadlocks:  Debug output generates USB debug
          output which generatates USB debug output, etc.  If you want USB
          debug output, you should consider enabling USB trace
          (CONFIG_USBDEV_TRACE) and perhaps the USB monitor (CONFIG_SYSTEM_USBMONITOR).

          See the usbnsh configuration below for more information on configuring
          USB trace output and the USB monitor.

   10. USB OTG FS Host Support.  The following changes will enable support for
       a USB host on the STM32F746G-DISCO, including support for a mass storage
       class driver:

       Device Drivers ->
         CONFIG_USBDEV=n          : Make sure tht USB device support is disabled
         CONFIG_USBHOST=y         : Enable USB host support
         CONFIG_USBHOST_ISOC_DISABLE=y

       Device Drivers -> USB Host Driver Support
         CONFIG_USBHOST_MSC=y     : Enable the mass storage class

       System Type -> STM32 Peripheral Support
         CONFIG_STM32F7_OTGHS=y     : Enable the STM32 USB OTG FH block (FS mode)
         CONFIG_STM32F7_SYSCFG=y    : Needed for all USB OTF HS support

       RTOS Features -> Work Queue Support
         CONFIG_SCHED_WORKQUEUE=y : High priority worker thread support is required
         CONFIG_SCHED_HPWORK=y    :   for the mass storage class driver.

       File Systems ->
         CONFIG_FS_FAT=y          : Needed by the USB host mass storage class.

       Board Selection ->
         CONFIG_LIB_BOARDCTL=y    : Needed for CONFIG_NSH_ARCHINIT

       Application Configuration -> NSH Library
         CONFIG_NSH_ARCHINIT=y    : Architecture specific USB initialization
                                  : is needed for NSH

       With those changes, you can use NSH with a FLASH pen driver as shown
       belong.  Here NSH is started with nothing in the USB host slot:

       NuttShell (NSH) NuttX-x.yy
       nsh> ls /dev
       /dev:
        console
        null
        ttyS0

       After inserting the FLASH drive, the /dev/sda appears and can be
       mounted like this:

       nsh> ls /dev
       /dev:
        console
        null
        sda
        ttyS0
       nsh> mount -t vfat /dev/sda /mnt/stuff
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c

       And files on the FLASH can be manipulated to standard interfaces:

       nsh> echo "This is a test" >/mnt/stuff/atest.txt
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c
        -rw-rw-rw-      16 atest.txt
       nsh> cat /mnt/stuff/atest.txt
       This is a test
       nsh> cp /mnt/stuff/filea.c fileb.c
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c
        -rw-rw-rw-      16 atest.txt
        -rw-rw-rw-   16236 fileb.c

       To prevent data loss, don't forget to un-mount the FLASH drive
       before removing it:

       nsh> umount /mnt/stuff

   11. I used this configuration to test the USB hub class.  I did this
       testing with the following changes to the configuration (in addition
       to those listed above for base USB host/mass storage class support):

       Drivers -> USB Host Driver Support
         CONFIG_USBHOST_HUB=y     : Enable the hub class
         CONFIG_USBHOST_ASYNCH=y  : Asynchonous I/O supported needed for hubs

       Board Selection ->
         CONFIG_STM32F746GDISCO_USBHOST_STACKSIZE=2048 (bigger than it needs to be)

       RTOS Features -> Work Queue Support
         CONFIG_SCHED_LPWORK=y     : Low priority queue support is needed
         CONFIG_SCHED_LPNTHREADS=1
         CONFIG_SCHED_LPWORKSTACKSIZE=1024

       NOTES:

       1. It is necessary to perform work on the low-priority work queue
          (vs. the high priority work queue) because deferred hub-related
          work requires some delays and waiting that is not appropriate on
          the high priority work queue.

       2. Stack usage make increase when USB hub support is enabled because
          the nesting depth of certain USB host class logic can increase.
