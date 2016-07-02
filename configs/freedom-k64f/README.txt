README.txt
==========

This is the README file for the port of NuttX to the Freescale Freedom-K64F
develoment board.

Contents
========

  o Freedom K64F Features
  o Serial Console
  o LEDs and Buttons
  o Development Environment
  o GNU Toolchain Options

Kinetis TWR-K60N512 Features:
=============================

  The features of the FRDM-K64F hardware are as follows:

  - MK64FN1M0VLL12 MCU (120 MHz, 1 MB flash memory, 256 KB RAM, low-power,
    crystal-less USB, and 100 LQFP)
  - Dual role USB interface with micro-B USB connector
  - RGB LED
  - FXOS8700CQ - accelerometer and magnetometer
  - Two user push buttons
  - Flexible power supply option - OpenSDAv2 USB, K64 USB, and external
    source
  - Easy access to MCU input/output through Arduino R3TM compatible I/O
    connectors
  - Programmable OpenSDAv2 debug circuit supporting the CMSIS-DAP Interface
    software that provides:
    o Mass storage device (MSD) flash programming interface
    o CMSIS-DAP debug interface over a driver-less USB HID connection
       providing run-control debugging and compatibility with IDE tools
    o Virtual serial port interface
    o Open-source CMSIS-DAP software project: github.com/mbedmicro/CMSIS-DAP.
  - Ethernet
  - SDHC
  - Add-on RF module: nRF24L01+ Nordic 2.4GHz Radio
  - Add-on Bluetooth module: JY-MCU BT board V1.05 BT

OpenSDAv2
=========

  The FRDM-K64F platform features OpenSDAv2, the Freescale open-source
  hardware embedded serial and debug adapter running an open-source
  bootloader. This circuit offers several options for serial communication,
  flash programming, and run-control debugging. OpenSDAv2 is an mbed
  HDK-compatible debug interface preloaded with the open-source CMSIS-DAP
  Interface firmware (mbed interface) for rapid prototyping and product
  development.

Serial Console
==============

  The primary serial port interface signals are PTB16 UART1_RX and PTB17
  UART1_TX. These signals are connected to the OpenSDAv2 circuit.

LEDs and Buttons
================

  RGB LED
  -------
  An RGB LED is connected through GPIO as shown below:

    LED    K64
    ------ -------------------------------------------------------
    RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
    BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
    GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN

  If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board the
  Freedom KL25Z.  Usage of these LEDs is defined in include/board.h and
  src/k64_leds.c.  The following definitions describe how NuttX controls the LEDs:

    SYMBOL                Meaning                 LED state
                                                  RED   GREEN  BLUE
    -------------------  -----------------------  -----------------
    LED_STARTED          NuttX has been started    OFF  OFF  OFF
    LED_HEAPALLOCATE     Heap has been allocated   OFF  OFF  ON
    LED_IRQSENABLED      Interrupts enabled        OFF  OFF  ON
    LED_STACKCREATED     Idle stack created        OFF  ON   OFF
    LED_INIRQ            In an interrupt          (no change)
    LED_SIGNAL           In a signal handler      (no change)
    LED_ASSERTION        An assertion failed      (no change)
    LED_PANIC            The system has crashed    FLASH OFF OFF
    LED_IDLE             K64 is in sleep mode     (Optional, not used)

  Buttons
  -------
  Two push buttons, SW2 and SW3, are available on FRDM-K64F board, where
  SW2 is connected to PTC6 and SW3 is connected to PTA4. Besides the
  general purpose input/output functions, SW2 and SW3 can be low-power
  wake up signal. Also, only SW3 can be a non-maskable interrupt.

  Switch    GPIO Function
  --------- ---------------------------------------------------------------
  SW2       PTC6/SPI0_SOUT/PD0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK/LLWU_P10
  SW3       PTA4/FTM0_CH1/NMI_b/LLWU_P3

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system supports several GNU-based toolchains under Linux,
  Cygwin under Windows, and Windoes native.  To select a toolchain:

  1. Use 'make menuconfig' and select the toolchain that you are using
     under the System Type menu.
  2. The default toolchain is the NuttX buildroot under Linux or Cygwin:

    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y

     If you are not using CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT, then you may
     also have to modify the PATH in the setenv.h file if your make cannot
     find the tools.

  NOTE:  Using native Windows toolchains under Cygwin has some limitations.
  This incuudes the CodeSourcery (for Windows) and devkitARM toolchains are
  Windows native toolchains.  The biggest limitations are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

Freedom K64F Configuration Options
==================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This sould
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=kinetis

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_MK64FN1M0VLL12

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD="freedom-k64f" (for the Freedom K64F development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_FREEDOM_K64F=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_RAM_SIZE=0x00010000 (64Kb)

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

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    CONFIG_KINETIS_TRACE    -- Enable trace clocking on power up.
    CONFIG_KINETIS_FLEXBUS  -- Enable flexbus clocking on power up.
    CONFIG_KINETIS_UART0    -- Support UART0
    CONFIG_KINETIS_UART1    -- Support UART1
    CONFIG_KINETIS_UART2    -- Support UART2
    CONFIG_KINETIS_UART3    -- Support UART3
    CONFIG_KINETIS_UART4    -- Support UART4
    CONFIG_KINETIS_UART5    -- Support UART5
    CONFIG_KINETIS_ENET     -- Support Ethernet (K60 only)
    CONFIG_KINETIS_RNGB     -- Support the random number generator(K60 only)
    CONFIG_KINETIS_FLEXCAN0 -- Support FlexCAN0
    CONFIG_KINETIS_FLEXCAN1 -- Support FlexCAN1
    CONFIG_KINETIS_SPI0     -- Support SPI0
    CONFIG_KINETIS_SPI1     -- Support SPI1
    CONFIG_KINETIS_SPI2     -- Support SPI2
    CONFIG_KINETIS_I2C0     -- Support I2C0
    CONFIG_KINETIS_I2C1     -- Support I2C1
    CONFIG_KINETIS_I2S      -- Support I2S
    CONFIG_KINETIS_DAC0     -- Support DAC0
    CONFIG_KINETIS_DAC1     -- Support DAC1
    CONFIG_KINETIS_ADC0     -- Support ADC0
    CONFIG_KINETIS_ADC1     -- Support ADC1
    CONFIG_KINETIS_CMP      -- Support CMP
    CONFIG_KINETIS_VREF     -- Support VREF
    CONFIG_KINETIS_SDHC     -- Support SD host controller
    CONFIG_KINETIS_FTM0     -- Support FlexTimer 0
    CONFIG_KINETIS_FTM1     -- Support FlexTimer 1
    CONFIG_KINETIS_FTM2     -- Support FlexTimer 2
    CONFIG_KINETIS_LPTIMER  -- Support the low power timer
    CONFIG_KINETIS_RTC      -- Support RTC
    CONFIG_KINETIS_SLCD     -- Support the segment LCD (K60 only)
    CONFIG_KINETIS_EWM      -- Support the external watchdog
    CONFIG_KINETIS_CMT      -- Support Carrier Modulator Transmitter
    CONFIG_KINETIS_USBOTG   -- Support USB OTG (see also CONFIG_USBHOST and CONFIG_USBDEV)
    CONFIG_KINETIS_USBDCD   -- Support the USB Device Charger Detection module
    CONFIG_KINETIS_LLWU     -- Support the Low Leakage Wake-Up Unit
    CONFIG_KINETIS_TSI      -- Support the touch screeen interface
    CONFIG_KINETIS_FTFL     -- Support FLASH
    CONFIG_KINETIS_DMA      -- Support DMA
    CONFIG_KINETIS_CRC      -- Support CRC
    CONFIG_KINETIS_PDB      -- Support the Programmable Delay Block
    CONFIG_KINETIS_PIT      -- Support Programmable Interval Timers
    CONFIG_ARM_MPU          -- Support the MPU

  Kinetis interrupt priorities (Default is the mid priority).  These should
  not be set because they can cause unhandled, nested interrupts.  All
  interrupts need to be at the default priority in the current design.

    CONFIG_KINETIS_UART0PRIO
    CONFIG_KINETIS_UART1PRIO
    CONFIG_KINETIS_UART2PRIO
    CONFIG_KINETIS_UART3PRIO
    CONFIG_KINETIS_UART4PRIO
    CONFIG_KINETIS_UART5PRIO

    CONFIG_KINETIS_EMACTMR_PRIO
    CONFIG_KINETIS_EMACTX_PRIO
    CONFIG_KINETIS_EMACRX_PRIO
    CONFIG_KINETIS_EMACMISC_PRIO

    CONFIG_KINETIS_SDHC_PRIO

  PIN Interrupt Support

    CONFIG_GPIO_IRQ          -- Enable pin interrupt support.  Also needs
      one or more of the following:
    CONFIG_KINETIS_PORTAINTS -- Support 32 Port A interrupts
    CONFIG_KINETIS_PORTBINTS -- Support 32 Port B interrupts
    CONFIG_KINETIS_PORTCINTS -- Support 32 Port C interrupts
    CONFIG_KINETIS_PORTDINTS -- Support 32 Port D interrupts
    CONFIG_KINETIS_PORTEINTS -- Support 32 Port E interrupts

  Kinetis K60 specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn (n=0..5) for the
      console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.
    CONFIG_UARTn_BITS - The number of bits.  Must be either 8 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

  Kenetis ethernet controller settings

    CONFIG_ENET_NRXBUFFERS - Number of RX buffers.  The size of one
        buffer is determined by CONFIG_NET_ETH_MTU.  Default: 6
    CONFIG_ENET_NTXBUFFERS - Number of TX buffers.  The size of one
        buffer is determined by CONFIG_NET_ETH_MTU.  Default: 2
    CONFIG_ENET_USEMII - Use MII mode.  Default: RMII mode.
    CONFIG_ENET_PHYADDR - PHY address

Configurations
==============

Each TWR-K60N512 configuration is maintained in a sub-directory and
can be selected as follow:

    cd tools
    ./configure.sh twr-k60n512/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.
    Support for the board's SPI-based MicroSD card is included
    (but not passing tests as of this writing).

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : The older OABI version
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

    3. An SDHC driver is under work and can be enabled in the NSH configuration
       for further testing be setting the following configuration values as
       follows:

      CONFIG_KINETIS_SDHC=y                : Enable the SDHC driver

      CONFIG_MMCSD=y                       : Enable MMC/SD support
      CONFIG_MMCSD_SDIO=y                  : Use the SDIO-based MMC/SD driver
      CONFIG_MMCSD_NSLOTS=1                : One MMC/SD slot

      CONFIG_FAT=y                         : Eable FAT file system
      CONFIG_FAT_LCNAMES=y                 : FAT lower case name support
      CONFIG_FAT_LFN=y                     : FAT long file name support
      CONFIG_FAT_MAXFNAME=32               : Maximum lenght of a long file name

      CONFIG_GPIO_IRQ=y                    : Enable GPIO interrupts
      CONFIG_KINETIS_PORTEINTS=y           : Enable PortE GPIO interrupts

      CONFIG_SCHED_WORKQUEUE=y             : Enable the NuttX workqueue

      CONFIG_NSH_ARCHINIT=y                : Provide NSH initializeation logic
