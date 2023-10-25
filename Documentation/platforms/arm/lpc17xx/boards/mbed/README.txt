README
^^^^^^

README for NuttX port to the mbed.org LPC1768 board (http://mbed.org/)

Contents
^^^^^^^^

  mbed Configuration Options
  USB Host Configuration
  Configurations

mbed Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^

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

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=mbed (for the mbed.org board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_MBED=y

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

    Individual subsystems can be enabled:
      CONFIG_LPC17_40_MAINOSC=y
      CONFIG_LPC17_40_PLL0=y
      CONFIG_LPC17_40_PLL1=n
      CONFIG_LPC17_40_ETHERNET=n
      CONFIG_LPC17_40_USBHOST=n
      CONFIG_LPC17_40_USBOTG=n
      CONFIG_LPC17_40_USBDEV=n
      CONFIG_LPC17_40_UART0=y
      CONFIG_LPC17_40_UART1=n
      CONFIG_LPC17_40_UART2=n
      CONFIG_LPC17_40_UART3=n
      CONFIG_LPC17_40_CAN1=n
      CONFIG_LPC17_40_CAN2=n
      CONFIG_LPC17_40_SPI=n
      CONFIG_LPC17_40_SSP0=n
      CONFIG_LPC17_40_SSP1=n
      CONFIG_LPC17_40_I2C0=n
      CONFIG_LPC17_40_I2C1=n
      CONFIG_LPC17_40_I2S=n
      CONFIG_LPC17_40_TMR0=n
      CONFIG_LPC17_40_TMR1=n
      CONFIG_LPC17_40_TMR2=n
      CONFIG_LPC17_40_TMR3=n
      CONFIG_LPC17_40_RIT=n
      CONFIG_LPC17_40_PWM0=n
      CONFIG_LPC17_40_MCPWM=n
      CONFIG_LPC17_40_QEI=n
      CONFIG_LPC17_40_RTC=n
      CONFIG_LPC17_40_WDT=n
      CONFIG_LPC17_40_ADC=n
      CONFIG_LPC17_40_DAC=n
      CONFIG_LPC17_40_GPDMA=n
      CONFIG_LPC17_40_FLASH=n

  LPC17xx/LPC40xx specific device driver settings

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

  LPC17xx/LPC40xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_LPC17_40_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_40_CAN1
      is defined.
    CONFIG_LPC17_40_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_40_CAN2
      is defined.
    CONFIG_LPC17_40_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this
      number. (the CCLK frequency is divided by this number to get the CAN
      clock). Options = {1,2,4,6}. Default: 4.
    CONFIG_LPC17_40_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this
      number.  (the CCLK frequency is divided by this number to get the CAN
      clock).  Options = {1,2,4,6}. Default: 4.
    CONFIG_LPC17_40_CAN_TSEG1 - The number of CAN time quanta in segment 1.
      Default: 6
    CONFIG_LPC17_40_CAN_TSEG2 = the number of CAN time quanta in segment 2.
      Default: 7

  LPC17xx/LPC40xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC17_40_ETHERNET.

    CONFIG_ETH0_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_LPC17_40_PHY_AUTONEG - Enable auto-negotiation
    CONFIG_LPC17_40_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_LPC17_40_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_LPC17_40_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_LPC17_40_ETH_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_LPC17_40_ETH_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_LPC17_40_ETH_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG_FEATURES.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG_FEATURES.
    CONFIG_LPC17_40_ETH_HASH - Enable receipt of near-perfect match frames.
    CONFIG_LPC17_40_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_MCASTGROUP is selected.

  LPC17xx/LPC40xx USB Device Configuration

    CONFIG_LPC17_40_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events.
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC17_40_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC17_40_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC17_40_USBDEV_DMA
      Enable lpc17xx/lpc40xx-specific DMA support
    CONFIG_LPC17_40_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC17_40_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC17xx/LPC40xx USB Host Configuration

    CONFIG_LPC17_40_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_LP17_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_LP17_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_LPC17_40_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_LPC17_40_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_LPC17_40_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

    CONFIG_USERLED
    CONFIG_USERLED_LOWER
      Setup the generic driver to support the board leds, and so creates the
device into /dev/userleds

USB Host Configuration
^^^^^^^^^^^^^^^^^^^^^^

  The mbed board can be easily modified to support a USB host interface
  (Remember to add 2 resistors of 15K to D+ and D- pins).

  The NuttShell (NSH) mbed can also be modified in order to support USB
  host operations.  To make these modifications, do the following:

  1. First configure to build the NSH configuration from the top-level
     NuttX directory:

     ./configure mbed/nsh

  2. Then edit the top-level .config file to enable USB host.  Make the
     following changes using 'make menuconfig':

     System Type -> LPC17xx/LPC40xx Peripheral Support
       CONFIG_LPC17_40_USBHOST=y        : USB host controller driver support

     Device Drivers -> USB Host Driver Support
       CONFIG_USBHOST=y              : USB host support
       CONFIG_USBHOST_ISOC_DISABLE=y : Not needed
       CONFIG_USBHOST_MSC=y          : Mass storage class support

     Library Routines
       CONFIG_SCHED_WORKQUEUE=y      : Enable worker thread

  When this change is made, NSH should be extended to support USB flash
  devices.  When a FLASH device is inserted, you should see a device
  appear in the /dev (pseudo) directory.  The device name should be
  like /dev/sda, /dev/sdb, etc.  The USB mass storage device, is present
  it can be mounted from the NSH command line like:

     ls /dev
     mount -t vfat /dev/sda /mnt/flash

  Files on the connect USB flash device should then be accessible under
  the mountpoint /mnt/flash.

Configurations
^^^^^^^^^^^^^^

  General
  -------

  Each mbed configuration is maintained in a sub-directory and can be selected
  as follow:

    tools/configure.sh mbed:<subdir>

  Where <subdir> is one of the mbed subdirectories described in the
  following paragraph.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  Configuration Sub-directories
  -----------------------------

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces.  See notes
    above for enabling USB host support in this configuration.
