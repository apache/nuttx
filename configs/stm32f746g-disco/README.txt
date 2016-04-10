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
    console on USART6 and RS-232 shield.  Very few other drivers are in
    place yet.

  2015-07-20:  STM32 F7 Ethernet appears to be functional, but has had
    only light testing.

  2015-07-21:  Added a protected build version of the NSH configuration
    (called knsh).  That configuration is close:  It boots, but I get
    a hard fault each time I do the NSH "help" command.  Everything else
    works fine.  I am thinking this is a corrupted binary; I am thinking
    that there is a bad pointer in the command table.  But this is hard
    to prove but possible because the steps to produce and load the
    binary are awkward.

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
  http://www.nuttx.org/doku.php?id=wiki:howtos:port-drivers_stm32f7

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

       CONFIG_ARCH_CORTEXM7=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32f7

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F746=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs/ subdirectory and,
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
       serves no purpose other than it allows you to calibrate
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

  Common Configuration Information
  --------------------------------
  Each STM32F746G-DISCO configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh stm32f746g-disco/<subdir>
    cd -
    . ./setenv.sh

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat STM32F746G-DISCO\<subdir>

  Where <subdir> is one of the sub-directories listed below.

  NOTES:

    1. These configurations use the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, these configurations use the USART6 for the serial
       console.  Pins are configured to that RX/TX are available at
       pins D0 and D1 of the Arduion connectors.  This should be compatible
       with most RS-232 shields.

    3. All of these configurations are set up to build under Windows using the
       "GNU Tools for ARM Embedded Processors" that is maintained by ARM
       (unless stated otherwise in the description of the configuration).

         https://launchpad.net/gcc-arm-embedded

       As of this writing (2015-03-11), full support is difficult to find
       for the Cortex-M7, but is supported by at least this realeasse of
       the ARM GNU tools:

         https://launchpadlibrarian.net/192228215/release.txt

       Current (2105-07-31) setenv.sh file are configured to use this
       release:

       https://launchpadlibrarian.net/209776344/release.txt

       That toolchain selection can easily be reconfigured using
       'make menuconfig'.  Here are the relevant current settings:

       Build Setup:
         CONFIG_HOST_WINDOWS=y               : Window environment
         CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

       System Type -> Toolchain:
         CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : GNU ARM EABI toolchain

       NOTE: As of this writing, there are issues with using this tool at
       the -Os level of optimization.  This has not been proven to be a
       compiler issue (as least not one that might not be fixed with a
       well placed volatile qualifier).  However, in any event, it is
       recommend that you use not more that -O2 optimization.

Configuration Directories
-------------------------

  kostest:
  -------
    This is identical to the nsh configuration below except that NuttX is
    built as a kernel-mode, monolithic module and the user applications are
    built separately.  Is is recommended to use a special make command;
    not just 'make' but make with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  This actual works!
    However, for me it is very confusing so I prefer the above make command:
    Make the user-space binaries first (pass1), then make the kernel-space
    binaries (pass2)

    NOTES:

    1. At the end of the build, there will be several files in the top-level
       NuttX build directory:

       PASS1:
         nuttx_user.elf    - The pass1 user-space ELF file
         nuttx_user.hex    - The pass1 Intel HEX format file (selected in defconfig)
         User.map          - Symbols in the user-space ELF file

       PASS2:
         nuttx             - The pass2 kernel-space ELF file
         nuttx.hex         - The pass2 Intel HEX file (selected in defconfig)
         System.map        - Symbols in the kernel-space ELF file

    2. Combining .hex files.  If you plan to use the STM32 ST-Link Utility to
       load the .hex files into FLASH, then you need to combine the two hex
       files into a single .hex file.  Here is how you can do that.

       a. The 'tail' of the nuttx.hex file should look something like this
          (with my comments added):

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
          this (again with my comments added):

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
          file to produce a single combined hex file:

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the STM32 ST-Link tool.  The
       mbed interface does not seem to except .hex files, but you can
       also convert the .hex file to binary with this command:

         arm-none-eabi-objcopy.exe -I ihex -O binary combined.hex combined.bin

       If you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

  netnsh:
  ------
    This is a NetShell (NSH) very similar to the nsh configuration described
    below.  It differs in that it has networking enabled.

    NOTES:

    1. Both IPv4 and IPv6 protocoals are enabled.  Fixed IP addresses are
       used.  The default configurationi target has these IP address:

       IPv4: 10.0.0.2
       IPv6: fc00::2

       These are, of course, easily changes by reconfiguring via 'make
       menuconfig'

    2. UDP, TCIP/IP, ARP, ICMP, and ICMPv6 are also enabled.

    3. NSH offers several network oriented commands such as:  ipconfig,
       ifup, ifdown, ping, and ping6.

    4. Telnet sessions are supported.  You can start a Telnet session from
       any host on the network using a command like:

         $ telnet 10.0.0.2
         Trying 10.0.0.2...
         Connected to 10.0.0.2.
         Escape character is '^]'.

         NuttShell (NSH) NuttX-7.10
         nsh> help
         help usage:  help [-v] [<cmd>]

           [           dd          hexdump     mb          ping6       sleep
           ?           echo        ifconfig    mkdir       ps          test
           break       exec        ifdown      mkfifo      pwd         true
           cat         exit        ifup        mh          rm          uname
           cd          false       kill        mv          rmdir       unset
           cp          free        losetup     mw          set         usleep
           cmp         help        ls          ping        sh          xd

         Builtin Apps:
         nsh>

       Under either Linux or Cygwin

    5. The PHY address is either 0 or 1, depending on the state of the
       LAN8720 RXER/PHYAD0 when the hardware is reset.  That connects to the
       STM32 F7 via PG2. PG2 is not controlled but appears to result in a
       PHY address of 0.

  nsh:
  ---
    Configures the NuttShell (NSH) located at apps/examples/nsh.  The
    Configuration enables the serial interfaces on UART6.  Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.
