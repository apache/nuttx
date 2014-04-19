README
^^^^^^

README for NuttX port to the Zilogic's ZKIT-ARM-1769 [NXP-LPC1769]
board.

Contents
^^^^^^^^

  ZKit-ARM LPC1769 Board
  Development Environment
  GNU Toolchain Options
  NuttX buildroot Toolchain
  LEDs
  ZKit-ARM Configuration Options
  Configurations

Zilogic's ZKit-ARM-1769 Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Pin Description                      On Board       Connector
  -------------------------------- ---------------- -------------
  P0.0/RD1/TXD3/SDA1                   RD1            CAN1
  P0.1/TD1/RXD3/SCL1                   TD1
  P0.2/TXD0/AD0.7                      TXD0           COM0
  P0.3/RXD0/AD0.6                      RXD0
  P0.4/I2SRX_CLK/RD2/CAP2.0            GPIO0
  P0.5/I2SRX_WS/TD2/CAP2.1             GPIO1
  P0.6/I2SRX_SDA/SSEL1/MAT2.0          SSEL1          SPI
  P0.7/I2STX_CLK/SCK1/MAT2.1           SCK1
  P0.8/I2STX_WS/MISO1/MAT2.2           MISO1
  P0.9/I2STX_SDA/MOSI1/MAT2.3          MOSI1
  P0.10/TXD2/SDA2/MAT3.0               TXD2           COM2
  P0.11/RXD2/SCL2/MAT3.1               RXD2
  P0.15/TXD1/SCK0/SCK                  SD-SCK
  P0.16/RXD1/SSEL0/SSEL                SD-SSEL        SD-CARD
  P0.17/CTS1/MISO0/MISO                SD-MISO
  P0.18/DCD1/M0SI0/MOSI                SD-MOSI
  P0.19/DSR1/SDA1                      LED1
  P0.20/DTR1/SCL1                      DTR1           COM1
  P0.21/RI1/RD1                        NC             LED2
  P0.22/RTS1/TD1                       RTS1           COM1
  P0.23/AD0.0/I2SRX_CLK/CAP3.0         AD0
  P0.24/AD0.1/I2SRX_WS/CAP3.1          AD1            AIN
  P0.25/AD0.2/I2SRX_SDA/TXD3           AD2
  P0.26/AD0.3/AOUT/RXD3                AD3
  P0.27/SDA0/USB_SDA                   SDA0           I2C0
  P0.28/SCL0/USB_SCL                   SCL0
  P0.29/USB_D+                         USB-D+         USB
  P0.30/USB_D-                         USB-D-

  P1.0/ENET_TXD0                       ETH-TXD0
  P1.1/ENET_TXD1                       ETH-TXD1
  P1.4/ENET_TX_EN                      ETH-TXEN
  P1.8/ENET_CRS                        ETH-CRS
  P1.9/ENET_RXD0                       ETH-RXD0       ETH
  P1.10/ENET_RXD1                      ETH-RXD1
  P1.14/ENET_RX_ER                     ETH-RXER
  P1.15/ENET_REF_CLK                   ETH-REFCLK
  P1.16/ENET_MDC                       ETH-MDC
  P1.17/ENET_MDIO                      ETH-MDIO
  P1.18/USB_UP_LED/PWM1.1/CAP1.0       USB-UP-LED
  P1.19/MCOA0/nUSB_PPWR/CAP1.1         KEY1
  P1.20/MCFB0/PWM1.2/SCK0              LCD-SCK
  P1.21/MCABORT/PWM1.3/SSEL0           LCD-SSEL
  P1.22/MCOB0/USB_PWRD/MAT1.0          LCD-A0         LCD
  P1.23/MCFB1/PWM1.4/MISO0             NC
  P1.24/MCFB2/PWM1.5/MOSI0             LCD_MOSI
  P1.25/MCOA1/MAT1.1                   LCD-RST
  P1.26/MCOB1/PWM1.6/CAP0.0            LCD-AO
  P1.27/CLKOUT/nUSB_OVRCR/CAP0.1       KEY2
  P1.28/MCOA2/MAT0.0                   KEY3
  P1.29/MCOB2/PCAP1.1/MAT0.1           CAP1           PWM-CON
  P1.30/VBUS/AD0.4                     VBUS           USB
  P1.31/SCK1/AD0.5                     KEY4

  P2.0/PWM1.1/TXD1                     TXD1
  P2.1/PWM1.2/RXD1                     RXD1           COM1
  P2.2/PWM1.3/CTS1/TRACEDATA3          CTS1
  P2.3/PWM1.4/DCD1/TRACEDATA2          PWM4
  P2.4/PWM1.5/DSR1/TRACEDATA1          PWM5           PWM
  P2.5/PWM1.6/DTR1/TRACEDATA0          PWM6
  P2.6/PCAP1.0/RI1/TRACECLK            CAP0
  P2.7/RD2/RTS1                        RD2            CAN2
  P2.8/TD2/TXD2                        TD2
  P2.9/USB_CONNECT/RXD2                USB_CONNECT    USB
  P2.10/nEINT0/NMI                     ISP
  P2.11/nEINT1/I2STX_CLK               INT1           I2C
  P2.12/nEINT2/I2STX_WS                SD-DET         SD-CARD
  P2.13/nEINT3/I2STX_SDA               KEY5

  P3.25/MAT0.0/PWM1.2                  PWM2           PWM
  P3.26/STCLK/MAT0.1/PWM1.3            PWM3

  P4.28/RX_MCLK/MAT2.0/TXD3            GPIO2          SPI
  P4.28/RX_MCLK/MAT2.0/TXD3            GPIO3

Zilogic's ZKit-ARM-1769 Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

SD Slot
-------

       Board        LPC1768
  SD   Signal       Pin
  ---  -----------  ----------
  CS   SD-SSEL      P0.16 SSEL0
  DIN  SD-MOSI      P0.18 MOSI0
  DOUT SD-MISO      P0.17 MISO0
  CLK  SD-SCK       P0.15 SCK0
  CD   SD-DET       P2.12

USB Device
----------

  Board             LPC1768
  Signal            Pin
  ----------------- -----------------
  USB_CONNECT       P2.9  USB_CONNECT
  USB_DM            P0.29 USB_D-
  USB_DP            P0.30 USB_D+
  USB_VBUS          P1.30 USB_VBUS
  USB_UPLED         P1.18 USB_UPLED

128x64 LCD with SPI interface
---------------------------------------
  The LCD display is connected to the SPI-bus.

    ZKit-ARM Signals

      ----------------------------+--------------- -------------------------------------------
      LPC1758 Pin                 | Board Signal   Description
      ----------------------------+--------------- -------------------------------------------
      P1.20/MCFB0/PWM1.2/SCK0     |  LCD-SCK       LCD Clock signal (D6)
      P1.21/MCABORT/PWM1.3/SSEL0  |  LCD-SSEL      LCD Chip Select  (CSB)
      P1.22/MCOB0/USB_PWRD/MAT1.0 |  LCD-A0        LCD-A0 (A0)
      P1.23/MCFB1/PWM1.4/MISO0    |  N.C
      P1.24/MCFB2/PWM1.5/MOSI0    |  LCD-MOSI      LCD Data (D7)
      P1.25/MCOA1/MAT1.1          |  LCD-RST       LCD Reset (RSTB) - Resets Everything in LCD
      ----------------------------+--------------- -------------------------------------------

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The Code Red GNU toolchain
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the Code Red toolchain and the
  make system is setup to default to use the Code Red Linux toolchain.  To use
  the other toolchain, you simply need add one of the following configuration
  options to your .config (or defconfig) file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y     : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y     : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDW=y      : Code Red toolchain under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODEREDL=y      : Code Red toolchain under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y       : The Atollic toolchain

  You may also have to modify the PATH in the setenv.h file if your make cannot
  find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Code Red (for Windoes)
  are Windows native toolchains.  The CodeSourcey (for Linux), Code Red (for Linux)
  and NuttX buildroot toolchains are Cygwin and/or Linux native toolchains. There
  are several limitations to using a Windows based toolchain in a Cygwin
  environment.  The three biggest are:

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

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh zkit-arm-1769/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  are building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE: The cortexm3-defconfig-4.3.3 produces an older-style is OABI toolchain.
  There is another configuration, cortexm3-eabi-defconfig-4.6.3, that will
  build a newer, EABI, toolchain.  Unfortunately, the 4.6.3 EABI toolchain is
  not compatible with the NXFLAT tools.  See the top-level TODO file (under
  "Binary loaders") for more information about this problem. If you plan to
  use NXFLAT, please do not use the GCC 4.6.3 EABI toochain; instead use the
  GCC 4.3.3 OABI toolchain.

NXFLAT Toolchain
^^^^^^^^^^^^^^^^

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh zkit-arm-1769/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

LEDs
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the ZKit-ARM LEDs will be
  included in the build.  See:

  - configs/zkit-arm-1769/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/zkit-arm-1769/src/zkitarm_internal.h - GPIO settings for the LEDs.

  - configs/zkit-arm-1769/src/up_leds.c - LED control logic.

  The ZKit-ARM LPC1768 has a single LEDs (there are more on the Embedded Artists
  base board, but those are not controlled by NuttX).  Usage this single LED by NuttX
  is as follows:

  - The LED is not illuminated until the ZKit-ARM completes initialization.

    If the LED is stuck in the OFF state, this means that the ZKit-ARM did not
    complete initializeation.

  - Each time the OS enters an interrupt (or a signal) it will turn the LED OFF and
    restores its previous stated upon return from the interrupt (or signal).

    The normal state, after initialization will be a dull glow.  The brightness of
    the glow will be inversely related to the proportion of time spent within interrupt
    handling logic.  The glow may decrease in brightness when the system is very
    busy handling device interrupts and increase in brightness as the system becomes
    idle.

    Stuck in the OFF state suggests that that the system never completed
    initialization;  Stuck in the ON state would indicated that the system
    intialialized, but is not takint interrupts.

  - If a fatal assertion or a fatal unhandled exception occurs, the LED will flash
    strongly as a slow, 2Hz rate.

ZKit-ARM Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc17xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1768=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=zkit-arm-1769

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_ZKITARM=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

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
      CONFIG_LPC17_MAINOSC=y
      CONFIG_LPC17_PLL0=y
      CONFIG_LPC17_PLL1=n
      CONFIG_LPC17_ETHERNET=n
      CONFIG_LPC17_USBHOST=n
      CONFIG_LPC17_USBOTG=n
      CONFIG_LPC17_USBDEV=n
      CONFIG_LPC17_UART0=y
      CONFIG_LPC17_UART1=n
      CONFIG_LPC17_UART2=n
      CONFIG_LPC17_UART3=n
      CONFIG_LPC17_CAN1=n
      CONFIG_LPC17_CAN2=n
      CONFIG_LPC17_SPI=n
      CONFIG_LPC17_SSP0=n
      CONFIG_LPC17_SSP1=n
      CONFIG_LPC17_I2C0=n
      CONFIG_LPC17_I2C1=n
      CONFIG_LPC17_I2S=n
      CONFIG_LPC17_TMR0=n
      CONFIG_LPC17_TMR1=n
      CONFIG_LPC17_TMR2=n
      CONFIG_LPC17_TMR3=n
      CONFIG_LPC17_RIT=n
      CONFIG_LPC17_PWM0=n
      CONFIG_LPC17_MCPWM=n
      CONFIG_LPC17_QEI=n
      CONFIG_LPC17_RTC=n
      CONFIG_LPC17_WDT=n
      CONFIG_LPC17_ADC=n
      CONFIG_LPC17_DAC=n
      CONFIG_LPC17_GPDMA=n
      CONFIG_LPC17_FLASH=n

  LPC17xx specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

  LPC17xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN2 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2. Default: 7

  LPC17xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC17_ETHERNET.

    CONFIG_ETH0_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_PHY_AUTONEG - Enable auto-negotion
    CONFIG_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_NET_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_NET_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_NET_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_NET_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG.
    CONFIG_NET_HASH - Enable receipt of near-perfect match frames.
    CONFIG_NET_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_IGMP is selected.

  LPC17xx USB Device Configuration

    CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events.
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC17_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC17_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC17_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC17_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC17_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC17xx USB Host Configuration (the ZKit-ARM does not support USB Host)

    CONFIG_USBHOST_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

Configurations
^^^^^^^^^^^^^^

Each ZKit-ARM configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh zkit-arm-1769/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  hello:
    This builds an example application using apps/examples/hello.  See
    apps/examples/README.txt for information about the examples.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default toolchain:

       CONFIG_HOST_LINUX=y                 : Builds under Windows (or Cygwin)
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : NuttX buildroot toolchain

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default toolchain:

       CONFIG_HOST_LINUX=y                 : Builds under Windows (or Cygwin)
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : NuttX buildroot toolchain

    3. TCP/IP (only) networking is enabled with this configuration:

       CONFIG_EXAMPLES_THTTPD_NOMAC=y             : Will use MAC 00:e0:de:ad:be:ef
       CONFIG_EXAMPLES_THTTPD_DRIPADDR=0xac100002 : Gateway 172.16.00.02
       CONFIG_EXAMPLES_THTTPD_NETMASK=0xffffff00  : Netmask 255.255.255.0

    4. You will need to build the NXFLAT toolchain as described above in
       order to use this example.
