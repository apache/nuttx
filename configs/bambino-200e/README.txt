README
======

README for NuttX port to the "Bambino 200E" board from Micromint USA
featuring the NXP LPC4330FBD144 MCU

Contents
========

  - Bambino 200E board
  - Status
  - Serial Console
  - FPU
  - Bambino-200e Configuration Options
  - Configurations

Bambino 200E board
=====================

  Memory Map
  ----------

  Block                 Start      Length
  Name                  Address
  --------------------- ---------- ------
  RAM                   0x10000000   128K
  RAM2                  0x10080000    72K
  RAMAHB                0x20000000    32K
  RAMAHB2               0x20008000    16K
  RAMAHB3               0x2000c000    16K
  SPIFI flash           0x1e000000  4096K

  GPIO Usage:
  -----------

  GPIO                              PIN     SIGNAL NAME
  -------------------------------- ------- --------------
  gpio3[7]  - LED1                  101     GPIO3[7]
  gpio5[5]  - LED2                  91      GPIO5[5]
  gpio0[7]  - BTN1                  96      GPIO0[7]

  Console
  -------

  The Bambino 200E default console is the UART1 on Gadgeteer Sockets 5 (U).

Status
======

  Many drivers are working (USB0 Device, Ethernet, etc), but many drivers are
  missing.

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

Serial Console
==============

The LPC4330 Xplorer does not have RS-232 drivers or serial connectors on board.
USART0 and UART1 are available on J8 as follows:

  ------ ------ -----------------------
  SIGNAL J8 PIN   LPC4330FET100 PIN
                  (TFBGA100 package)
  ------ ------ -----------------------
  U0_TXD pin  9  F6  P6_4  U0_TXD=Alt 2
  U0_RXD pin 10  F9  P6_5  U0_RXD=Alt 2
  U1_TXD pin 13  H8  P1_13 U1_TXD=Alt 1
  U1_RXD pin 14  J8  P1_14 U1_RXD=Alt 1
  ------ ------ -----------------------

  GND  is available on J8 pin 1
  5V   is available on J8 pin 2
  VBAT is available on J8 pin 3

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the most NuttX Cortex-M4
ports.

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
     CONFIG_ARMV7M_LAZYFPU=y

CFLAGS
------

Only the recent toolchains have built-in support for the Cortex-M4 FPU.  You will see
the following lines in each Make.defs file:

  ifeq ($(CONFIG_ARCH_FPU),y)
    ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  else
    ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
  endif

Configuration Changes
---------------------

Below are all of the configuration changes that I had to make to configs/stm3240g-eval/nsh2
in order to successfully build NuttX using the Atollic toolchain WITH FPU support:

  -CONFIG_ARCH_FPU=n                       : Enable FPU support
  +CONFIG_ARCH_FPU=y

  -CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : Disable the CodeSourcery toolchain
  +CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=n

  -CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=n       : Enable the Atollic toolchains
  +CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y       :

  -CONFIG_INTELHEX_BINARY=y                : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=n                : (Only necessary with the "Lite" version)

  -CONFIG_HAVE_CXX=y                       : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=n                       : (Only necessary with the "Lite" version)

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

Bambino-200e Configuration Options
==================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc43xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC4330=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=bambino-200e (for the Bambino-200e board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_BAMBINO_200E=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_RAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_RAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_RAM_START - The start address of installed DRAM

       CONFIG_RAM_START=0x10000000

    CONFIG_ARCH_FPU - The LPC43xxx supports a floating point unit (FPU)

       CONFIG_ARCH_FPU=y

    CONFIG_LPC43_BOOT_xxx - The startup code needs to know if the code is running
       from internal FLASH, external FLASH, SPIFI, or SRAM in order to
       initialize properly.  Note that a boot device is not specified for
       cases where the code is copied into SRAM; those cases are all covered
       by CONFIG_LPC43_BOOT_SRAM.

       CONFIG_LPC43_BOOT_SRAM=y      : Running from SRAM             (0x1000:0000)
       CONFIG_LPC43_BOOT_SPIFI=y     : Running from QuadFLASH        (0x1400:0000)
       CONFIG_LPC43_BOOT_FLASHA=y    : Running in internal FLASHA    (0x1a00:0000)
       CONFIG_LPC43_BOOT_FLASHB=y    : Running in internal FLASHA    (0x1b00:0000)
       CONFIG_LPC43_BOOT_CS0FLASH=y  : Running in external FLASH CS0 (0x1c00:0000)
       CONFIG_LPC43_BOOT_CS1FLASH=y  : Running in external FLASH CS1 (0x1d00:0000)
       CONFIG_LPC43_BOOT_CS2FLASH=y  : Running in external FLASH CS2 (0x1e00:0000)
       CONFIG_LPC43_BOOT_CS3FLASH=y  : Running in external FLASH CS3 (0x1f00:0000)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    Individual subsystems can be enabled:

      CONFIG_LPC43_ADC0=y
      CONFIG_LPC43_ADC1=y
      CONFIG_LPC43_ATIMER=y
      CONFIG_LPC43_CAN0=y
      CONFIG_LPC43_CAN1=y
      CONFIG_LPC43_DAC=y
      CONFIG_LPC43_EMC=y
      CONFIG_LPC43_ETHERNET=y
      CONFIG_LPC43_EVNTMNTR=y
      CONFIG_LPC43_GPDMA=y
      CONFIG_LPC43_I2C0=y
      CONFIG_LPC43_I2C1=y
      CONFIG_LPC43_I2S0=y
      CONFIG_LPC43_I2S1=y
      CONFIG_LPC43_LCD=y
      CONFIG_LPC43_MCPWM=y
      CONFIG_LPC43_QEI=y
      CONFIG_LPC43_RIT=y
      CONFIG_LPC43_RTC=y
      CONFIG_LPC43_SCT=y
      CONFIG_LPC43_SDMMC=y
      CONFIG_LPC43_SPI=y
      CONFIG_LPC43_SPIFI=y
      CONFIG_LPC43_SSP0=y
      CONFIG_LPC43_SSP1=y
      CONFIG_LPC43_TMR0=y
      CONFIG_LPC43_TMR1=y
      CONFIG_LPC43_TMR2=y
      CONFIG_LPC43_TMR3=y
      CONFIG_LPC43_USART0=y
      CONFIG_LPC43_UART1=y
      CONFIG_LPC43_USART2=y
      CONFIG_LPC43_USART3=y
      CONFIG_LPC43_USB0=y
      CONFIG_LPC43_USB1=y
      CONFIG_LPC43_USB1_ULPI=y
      CONFIG_LPC43_WWDT=y

  LPC43xx specific U[S]ART device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the USART0).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

    CONFIG_USARTn_RS485MODE - Support LPC43xx USART0,2,3 RS485 mode
      ioctls (TIOCSRS485 and TIOCGRS485) to enable and disable
      RS-485 mode.

  LPC43xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_LPC43_CAN0_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN0
      is defined.
    CONFIG_LPC43_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN1
      is defined.
    CONFIG_LPC43_CAN_TSEG1 - The number of CAN time quanta in segment 1.
      Default: 12
    CONFIG_LPC43_CAN_TSEG2 = the number of CAN time quanta in segment 2.
      Default: 4

  LPC43xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC43_ETHERNET.

    CONFIG_ETH0_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_LPC43_AUTONEG - Enable auto-negotion

    CONFIG_LPC17_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_LPC43_ETH_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_LPC43_ETH_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG_FEATURES.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG_FEATURES.

  LPC43xx USB Device Configuration

    CONFIG_LPC43_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events.
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC43_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC43_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC43_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC43_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC43_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

Configurations
==============

Each Bambino-200e configuration is maintained in a sub-directory and can be selected
as follow:

    tools/configure.sh bambino-200e/<subdir>

Where <subdir> is one of the following:

  knsh:
  -----

    This is identical to the nsh configuration below except that NuttX
    is built as a PROTECTED mode, monolithic module and the user applications
    are built separately.

    It is recommends to use a special make command; not just 'make' but make
    with the following two arguments:

        make pass1 pass2

    In the normal case (just 'make'), make will attempt to build both user-
    and kernel-mode blobs more or less interleaved.  That actual works!
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

       The J-Link programmer will except files in .hex, .mot, .srec, and .bin
       formats.

    2. Combining .hex files.  If you plan to use the .hex files with your
       debugger or FLASH utility, then you may need to combine the two hex
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

          $ cat nuttx.hex nuttx_user.hex >combined.hex

       Then use the combined.hex file with the to write the FLASH image.
       If you do this a lot, you will probably want to invest a little time
       to develop a tool to automate these steps.

    Other option is to combine nuttx.bin and nuttx_user.bin this way:

      $ dd if=/dev/zero of=empty.bin bs=1k count=256
      $ cat nuttx.bin empty.bin > nuttxtmp.bin
      $ dd if=nuttxtmp.bin of=nuttxpad.bin bs=1k count=256
      $ cat nuttxpad.bin nuttx_user.bin > nuttxfinal.bin

  netnsh:
  -------
    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on network testing.

  nsh:
  ----
    This configuration is the NuttShell (NSH) example at examples/nsh/.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. By default, this project assumes that you are executing directly from
       SRAM.

         CONFIG_LPC43_BOOT_SRAM=y           : Executing in SRAM
         CONFIG_ARMV7M_TOOLCHAIN_CODEREDW=y : Code Red under Windows

    3. To execute from SPIFI, you would need to set:

         CONFIG_LPC43_BOOT_SPIFI=y      : Executing from SPIFI
         CONFIG_RAM_SIZE=(128*1024)     : SRAM Bank0 size
         CONFIG_RAM_START=0x10000000    : SRAM Bank0 base address
         CONFIG_SPIFI_OFFSET=(512*1024) : SPIFI file system offset

       CONFIG_MM_REGIONS should also be increased if you want to other SRAM banks
       to the memory pool.

    4. This configuration an also be used create a block device on the SPIFI
       FLASH.  CONFIG_LPC43_SPIFI=y must also be defined to enable SPIFI setup
       support:

       SPIFI device geometry:

         CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
           bytes into the device address space.  This offset must be an exact
           multiple of the erase block size (CONFIG_SPIFI_BLKSIZE). Default 0.
         CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not defined
           then the driver will try to determine the correct erase block size by
           examining that data returned from spifi_initialize (which sometimes
           seems bad).

       Other SPIFI options

         CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
           FAT friendly 512 byte sector size and will manage the read-modify-write
           operations on the larger erase block.
         CONFIG_SPIFI_READONLY - Define to support only read-only operations.
         CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
           use an external library implementation of the SPIFI interface.
         CONFIG_SPIFI_VERIFY - Verify all spifi_program() operations by reading
           from the SPI address space after each write.
         CONFIG_DEBUG_SPIFI_DUMP - Debug option to dump read/write buffers.  You
           probably do not want to enable this unless you want to dig through a
           *lot* of debug output!  Also required CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO,
           and CONFIG_DEBUG_FS,

    5. In my experience, there were some missing function pointers in the LPC43xx
       SPIFI ROM routines and the SPIFI configuration could only be built with
       CONFIG_SPIFI_LIBRARY=y.  The SPIFI library is proprietary and cannot be
       provided within NuttX open source repository; SPIFI library binaries can
       be found on the lpcware.com website.  In this build sceneario, you must
       also provide the patch to the external SPIFI library be defining the make
       variable EXTRA_LIBS in the top-level Make.defs file.  Good luck!

  usbnsh:
  -------

    This is another NSH example.  If differs from other 'nsh' configurations
    in that this configurations uses a USB serial device for console I/O.

    NOTES:

    1. This configuration does have UART1 output enabled and set up as
       the system logging device:

       CONFIG_SYSLOG_CHAR=y               : Use a character device for system logging
       CONFIG_SYSLOG_DEVPATH="/dev/ttyS0" : UART1 will be /dev/ttyS0

       However, there is nothing to generate SYLOG output in the default
       configuration so nothing should appear on UART1 unless you enable
       some debug output or enable the USB monitor.

       NOTE:  Using the SYSLOG to get debug output has limitations.  Among
       those are that you cannot get debug output from interrupt handlers.
       So, in particularly, debug output is not a useful way to debug the
       USB device controller driver.  Instead, use the USB monitor with
       USB debug off and USB trace on (see below).

    4. Enabling USB monitor SYSLOG output.  If tracing is enabled, the USB
       device will save encoded trace output in in-memory buffer; if the
       USB monitor is enabled, that trace buffer will be periodically
       emptied and dumped to the system logging device (UART2 in this
       configuration):

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

