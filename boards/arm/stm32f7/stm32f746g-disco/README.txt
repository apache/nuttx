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

  - STATUS
  - Development Environment
  - LEDs and Buttons
  - Serial Console
  - Porting STM32 F4 Drivers
  - FPU
  - STM32F746G-DISCO-specific Configuration Options
  - Configurations

STATUS
======

  2015-07-19:  The basic NSH configuration is functional using a serial
    console on USART1 (Virtual COM, i.e. ttyACM0). Very few other drivers
    are in place yet.

  2015-07-20:  STM32 F7 Ethernet appears to be functional, but has had
    only light testing.

Development Environment
=======================

  The Development environments for the STM32F746G-DISCO board are identical
  to the environments for other STM32F boards.  For full details on the
  environment options and setup, see the README.txt file in the
  boards/arm/stm32f7/stm32f746g-disco directory.

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

  The STM32F469G-DISCO uses USART1 connected to "Virtual COM", so when you
  plug it on your computer it will be detected as a USB port (i.e. ttyACM0):

  -------- ---------------
              STM32F7
  V.COM     FUNCTION  GPIO
  -----    --------- -----
  RXD      USART1_RX PB7
  TXD      USART1_TX PA9
  ------   --------- -----

  All you need to do after flashing NuttX on this board is use a serial
  console tool (minicom, picocom, screen, hyperterminal, teraterm, putty,
  etc ) configured to 115200 8n1.

Porting STM32 F4 Drivers
========================

  The STM32F746 is very similar to the STM32 F429 and many of the drivers
  in the stm32/ directory could be ported here:  ADC, BBSRAM, CAN, DAC,
  DMA2D, FLASH, I2C, IWDG, LSE, LSI, LTDC, OTGFS, OTGHS, PM, Quadrature
  Encoder, RNG, RTCC, SDMMC (was SDIO), Timer/counters, and WWDG.

  Many of these drivers would be ported very simply; many ports would just
  be a matter of copying files and some seach-and-replacement.  Like:

    1. Compare the two register definitions files; make sure that the STM32
       F4 peripheral is identical (or nearly identical) to the F7
       peripheral.  If so then,
    2. Copy the register definition file from the stm32/chip directory to
       the stm32f7/chip directory, making name changes as appropriate and
       updating the driver for any minor register differences.
    3. Copy the corresponding C  file (and possibly a matching .h file) from
       the stm32/ directory to the stm32f7/ directory again with naming
       changes and changes for any register differences.
    4. Update the Make.defs file to include the new C file in the build.

  For other files, particularly those that use DMA, the port will be
  significantly more complex.  That is because the STM32F7 has a D-Cache
  and, as a result, we need to exercise much more care to maintain cache
  coherency.  There is a Wiki page discussing the issues of porting
  drivers from the stm32/ to the stm32f7/ directories here:
  https://cwiki.apache.org/confluence/display/NUTTX/Porting+Drivers+to+the+STM32+F7

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
   with:

     CONFIG_ARCH_FPU=y

2. Lazy Floating Point Register Save.

   An alternative mplementation only saves and restores FPU registers only
   on context switches.  This means: (1) floating point registers are not
   stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file:

     CONFIG_ARCH_FPU=y

STM32F746G-DISCO-specific Configuration Options
===============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM7=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32f7

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F746=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and,
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm32f746g-disco (for the STM32F746G-DISCO development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM32F746G_DISCO=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - should not be defined.

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

    CONFIG_RAM_START - The start address of installed SRAM (SRAM1)

       CONFIG_RAM_START=0x20010000
       CONFIG_RAM_SIZE=245760

    This configurations use only SRAM1 for data storage.  The heap includes
    the remainder of SRAM1.  If CONFIG_MM_REGIONS=2, then SRAM2 will be
    included in the heap.

    DTCM SRAM is never included in the heap because it cannot be used for
    DMA.  A DTCM allocator is available, however, so that DTCM can be
    managed with dtcm_malloc(), dtcm_free(), etc.

    In order to use FMC SRAM, the following additional things need to be
    present in the NuttX configuration file:

    CONFIG_STM32F7_FMC_SRAM - Indicates that SRAM is available via the
      FMC (as opposed to an LCD or FLASH).

    CONFIG_HEAP2_BASE - The base address of the SRAM in the FMC address space (hex)

    CONFIG_HEAP2_SIZE - The size of the SRAM in the FMC address space (decimal)

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
    CONFIG_STM32F7_OTGHS          USB OTG HS

    AHB2
    ----
    CONFIG_STM32F7_OTGFS          USB OTG FS
    CONFIG_STM32F7_DCMI           DCMI
    CONFIG_STM32F7_CRYP           CRYP
    CONFIG_STM32F7_HASH           HASH
    CONFIG_STM32F7_RNG            RNG

    AHB3
    ----

    CONFIG_STM32F7_FMC            FMC control registers
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
    CONFIG_STM32F7_CAN1_BAUD - CAN1 BAUD rate.  Required if
      CONFIG_STM32F7_CAN1 is defined.
    CONFIG_STM32F7_CAN2_BAUD - CAN1 BAUD rate.  Required if
      CONFIG_STM32F7_CAN2 is defined.
    CONFIG_STM32_CAN_TSEG1 - The number of CAN time quanta in segment 1.
      Default: 6
    CONFIG_STM32_CAN_TSEG2 - the number of CAN time quanta in segment 2.
      Default: 7
    CONFIG_STM32_CAN_REGDEBUG - If CONFIG_DEBUG_FEATURES is set, this will generate an
      dump of all CAN registers.

  STM32F746G-DISCO SPI Configuration

    CONFIG_STM32F7_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32F7_SPIx_DMA - Use DMA to improve SPIx transfer performance.
      Cannot be used with CONFIG_STM32F7_SPI_INTERRUPT.

  STM32F746G-DISCO DMA Configuration

    CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32F7_SDIO
      and CONFIG_STM32F7_DMA2.
    CONFIG_STM32_SDIO_DMAPRIO - Select SDIO DMA interrupt priority.
      Default:  Medium
    CONFIG_STM32_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
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
     debug.  Depends on CONFIG_DEBUG_FEATURES.
   CONFIG_STM32F7_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
     packets. Depends on CONFIG_DEBUG_FEATURES.

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each STM32F746G-DISCO configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh stm32f746g-disco:<subdir>

  Where <subdir> is one of the sub-directories listed below.

  NOTES:

    1. These configurations use the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, these configurations use the USART1 for the serial
       console.  Pins are configured to that RX/TX are available at
       pins D0 and D1 of the Arduion connectors.  This should be compatible
       with most RS-232 shields.

    3. All of these configurations are set up to build under Windows using the
       "GNU Tools for ARM Embedded Processors" that is maintained by ARM
       (unless stated otherwise in the description of the configuration).

         https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

       As of this writing (2015-03-11), full support is difficult to find
       for the Cortex-M7, but is supported by at least this release of
       the ARM GNU tools:

         https://launchpadlibrarian.net/209776344/release.txt

       That toolchain selection can easily be reconfigured using
       'make menuconfig'.  Here are the relevant current settings:

       Build Setup:
         CONFIG_HOST_WINDOWS=y               : Window environment
         CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

       System Type -> Toolchain:
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

       NOTE: As of this writing, there are issues with using this tool at
       the -Os level of optimization.  This has not been proven to be a
       compiler issue (as least not one that might not be fixed with a
       well placed volatile qualifier).  However, in any event, it is
       recommend that you use not more that -O2 optimization.

Configuration Directories
-------------------------

  nsh
  ---
    Configures the NuttShell (NSH) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on USART1.  Support for
    built-in applications is enabled, but in the base configuration no
    built-in applications are selected.

  netnsh
  ------
    This configuration is similar to the nsh but a lot more hardware
    peripherals are enabled, in particular Ethernet, as well as networking
    support.  It is similar to the stm32f769i-disco/netnsh
    configuration. This configuration uses USART1 for the serial console.
    USART1 is connected to the ST-link virtual com inside board.h to remove
    the need of a extra serial connection to use this board.

  lgvl
  ----
    STM32F746G-DISCO LittlevGL demo example.

    The LTDC is initialized during boot up.
    This configuration uses USART1 for the serial console.
    USART1 is connected to the ST-link virtual com inside board.h to remove
    the need of a extra serial connection to use this board.
    From the nsh command line execute the lvgldemo example:

      nsh> lvgldemo

    The test will execute the calibration process and then run the
    LittlevGL demo project.
