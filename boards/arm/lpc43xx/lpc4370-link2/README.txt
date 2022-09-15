README
======

README for NuttX port to the LPC4370-Link2 board featuring the NXP
LPC4370FET100 MCU - The port was derived from the LPC4370-Link2 board NuttX
port.

Contents
========

  - LPC4370-Link2 development board
  - Status
  - Code Red IDE/Tools
    Booting the LPCLink
    Using GDB
    Troubleshooting
    Command Line Flash Programming
    Executing from SPIFI
    USB DFU Booting
  - LED and Pushbuttons
  - Serial Console
  - FPU
  - LPC4370-Link2 Configuration Options
  - Configurations
  - STATUS

LPC4370-Link2 board
=================

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
  RAMM0                 0x18000000    16K
  RAMM01                0x18004000    2K
  SPIFI flash           0x14000000  1024K

  Console
  -------

  The LPC4370-Link2 default console is the USART2.

Status
======

  This is the current status of the LPC43xx port:

  - The basic OS test configuration and the basic NSH configurations
    are present and fully verified.  This includes:  SYSTICK system time,
    pin and GPIO configuration, and serial console support.  A SPIFI
    MTD driver is also in place but requires further verification.

  - The following drivers have been copied from the LPC17xx/LPC40xx port, but
    require integration into the LPC43xx.  This integration should
    consist of:

    - Remove LPC17xx/LPC40xx power, clocking, and pin configuration logic.
    - Adding of clock source and frequency to the board.h file.
    - Adding of LPC43 clock connection and pin configuration logic.

    Within any luck, these drivers should come up very quickly:

    - lpc43_adc.c,
    - lpc43_dac.c,
    - lpc43_gpdma.c,
    - lpc43_i2c.c,
    - lpc43_spi.c, and
    - lpc43_ssp.c

    These LPC17xx/LPC40xx drivers were not brought into the LPC43xx port because
    it appears that these peripherals have been completely redesigned:

    - CAN,
    - Ethernet,
    - USB device, and
    - USB host.

    The following LPC43xx peripherals are unsupported.  Some may be
    compatible with the LPC17xx/LPC40xx, but there is no LPC17xx/LPC40xx driver to be
    ported:

    - SD/MMC,
    - EMC,
    - USB0,
    - USB1,
    - Ethernet,
    - LCD,
    - SCT,
    - Timers 0-3
    - MCPWM,
    - QEI,
    - Alarm timer,
    - WWDT,
    - RTC,
    - Event monitor, and
    - CAN,

    For the missing drivers some of these can be leveraged from other
    MCUs that appear to support the same peripheral IP.

    - USB0 appears to be the same as the USB OTG peripheral for the
      LPC31xx.  It should be possible to drop in the LPC31xx driver
      with a small porting effort.

    - The Ethernet block looks to be based on the same IP as the
      STM32 Ethernet and, as a result, it should be possible to leverage
      the STM32 Ethernet driver with a little more effort.

Code Red IDE/Tools
^^^^^^^^^^^^^^^^^^

  Booting the LPCLink
  -------------------

  The first step is to activate the LPCLink's boot mode.  Some general
  instructions to do this are provided here:

    http://support.code-red-tech.com/CodeRedWiki/BootingLPCLink

  For my RedSuite installation path, that can be done using the following
  steps in a Cygwin bash shell:

    $ /cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin/Scripts/bootLPCXpresso.cmd winusb
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . .

  The same file logic can be found the less restrictive LPCXpresso package at:

    /cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin

  (The "free" RedSuite version has a download limit of 8K; the "free" LPCXpresso
  version has a download limit of 128K).

  NOTE that the following alias may be defined to enter the boot mode with a
  simpler command:

    alias lpc43xx='${SCRIPT_BIN}/Scripts/bootLPCXpresso.cmd winusb'

  You may also have to modify the PATH environment variable if your make cannot
  find the tools.

    $ lpc43xx
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . .

  Using GDB
  ---------

  The underlying debugger within Red Suite/LPCXpresso is GDB.  That GDB
  used from the command line.  The GDB configuration details for command
  line use are on Code Red Wiki:

    http://support.code-red-tech.com/CodeRedWiki/UsingGDB

  and is also summarized here (see the full Wiki for additional details
  and options).

  The Code Red Debug Driver implements the GDB "remote" protocol to allow
  connection to debug targets.  To start a debug session using GDB, use
  following steps:

    arm-none-eabi-gdb executable.axf                  : Start GDB and name the debug image
    target extended-remote | <debug driver> <options> : Start debug driver, connect to target
    load                                              : Load image and download to target

  The where <debug driver> is crt_emu_lpc18_43_nxp for LPC18xx and LPC43xx.
  Your PATH variable should be set up so that the debug driver executable
  can be found.  For my installation, the driver for the LPC18xx and LPC43xx
  is located at:

    /cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin/crt_emu_lpc18_43_nxp.exe, OR
    /cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin/crt_emu_lpc18_43_nxp.exe

  And <options> are:

    -n set information level for the debug driver. n should be 2, 3 or 4.
        2 should be sufficient in most circumstances

    -p<part> is the target device to connect to and you should use
        <part>=LPC4370.

    -wire=<probe> specifies the debug probe.  For LPCLink on Windows 7 use
        <probe>=winusb.  The 128K free version only supports the LPC-Link
        and RedProbe debug probes. Other JTAG interfaces are supported in
        the full version.

  Thus the correct invocation for the LPC4370 under Windows7 would be:

    target extended-remote | crt_emu_lpc18_43_nxp -2 -pLPC4370 -wire=winusb

  DDD.  This command can be used to start GDB under the graphics front-end
  DDD:

    $ ddd --debugger arm-none-eabi-gdb nuttx &

  NOTE 1:  Don't forget to put the LPCLink in boot mode as described above
  before starting GDB.  So a typical session might look like this:

    $ lpc43xx
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . .

    $ arm-none-eabi-gdb nuttx
    (gdb) target extended-remote | crt_emu_lpc18_43_nxp -2 -pLPC4370 -wire=winusb
    (gdb) load
    (gdb) r
    (gdb) c

  NOTE 2:  Don't forget to enable CONFIG_DEBUG_SYMBOLS=y in your NuttX
  configuration file when you build NuttX.  That option is necessary to build
  in debugging symbols.

  NOTE 3:  There are few things that NuttX has to do differently if you
  are using a debugger.  Make sure that you also set CONFIG_DEBUG_FEATURES=y.  Nothing
  also is needed and no debug output will be generated; but NuttX will
  use CONFIG_DEBUG_FEATURES=y to mean that a debugger is attached and will deal
  with certain resets and debug controls appropriately.

  So you should have:

    CONFIG_DEBUG_FEATURES=y
    CONFIG_DEBUG_SYMBOLS=y

  NOTE 4: Every time that you control-C out of the command line GDB, you
  leave a copy of the Code Red debugger (crt_emu_lpc18_43_nxp) running.  I
  have found that if you have these old copies of the debugger running,
  hen strange things can happen when start yet another copy of the
  debugger (I suspect that GDB may be talking with the wrong debugger).

  If you exit GDB with quit (not control-C), it seems to clean-up okay.
  But I have taken to keeping a Process Explorer window open all of the
  time to keep track of how many of these bad processes have been created.

  NOTE 5: There is also a certain function that is causing some problems.
  The very first thing that the start-up logic does is call a function
  called lpc43_softreset() which resets most of the peripherals.  But it
  also causes some crashes... I think because the resets are causing some
  interrupts.

  I put a big delay in the soft reset logic between resetting and clearing
  pending interrupts  and that seems to help some but I am not confident
  that that is a fix.  I think that the real fix might be to just eliminated
  this lpc43_softreset() function if we determine that it is not needed.

  If you step over lpc43_softreset() after loading the coding (using the 'n'
  command), then everything seems work okay.

  Troubleshooting
  ---------------

  This page provides some troubleshooting information that you can use to
  verify that the LPCLink is working correctly:

    http://support.code-red-tech.com/CodeRedWiki/LPCLinkDiagnostics

  Command Line Flash Programming
  ------------------------------

  The LPC18xx/LPC43xx debug driver can also be used to program the LPC43xx
  flash directly from the command line.  The script flash.sh that may be
  found in the boards/arm/lpc43xx/lpc4370-Link2/scripts directory can do that
  with a single command line command.

  Executing from SPIFI
  --------------------

  By default, the configurations here assume that you are executing directly
  from SRAM.

    CONFIG_LPC43_BOOT_SRAM=y            : Executing in SRAM
    CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU EABI toolchain for Windows

  To execute from SPIFI, you would need to set:

    CONFIG_LPC43_BOOT_SPIFI=y          : Executing from SPIFI
    CONFIG_RAM_SIZE=(128*1024)         : SRAM Bank0 size
    CONFIG_RAM_START=0x10000000        : SRAM Bank0 base address
    CONFIG_SPIFI_OFFSET=(512*1024)     : SPIFI file system offset

  To boot the LPC4370-Link2 from SPIFI the DIP switches should be 1-OFF,
  2-ON, 3-ON, 4-ON (LOW LOW LOW HIGH in Table 19, MSB to LSB).

  If the code in flash hard faults after reset and crt_emu_lpc18_43_nxp
  can't reset the MCU, an alternative is to temporarily change switch 1
  to  ON and press the reset button so it enters UART boot mode. Then
  change it back to OFF and reset to boot again from flash.

  # Use -wire to specify the debug probe in use:
  #   (empty)       Red Probe+
  #   -wire=winusb  LPC-Link on Windows XP
  #   -wire=hid     LPC-Link on Windows Vista/ Windows 7
  # Add -g -4 for verbose output

  crt_emu_lpc18_43_nxp -wire=hid -pLPC4370 -load-base=0x14000000
    -flash-load-exec=nuttx.bin -flash-driver=LPC1850A_4350A_SPIFI.cfx

  USB DFU Booting
  ---------------

  To be provided.

LED and Pushbuttons
===================

  LED
  ---
  The LPC4370-Link2 has one user-controllable LED labelled D6 controlled
  by the signal LED_3V3:

    LED SIGNAL  MCU
    D6 LED_3V3 PE_& GPIO7[7]

  A low output illuminates the LED.

  If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
  for NuttX debug functionality (where NC means "No Change").

    -------------------------- ---------
                               LED
    -------------------------- ---------
    LED_STARTED                OFF
    LED_HEAPALLOCATE           OFF
    LED_IRQSENABLED            OFF
    LED_STACKCREATED           ON
    LED_INIRQ                  NC
    LED_SIGNAL                 NC
    LED_ASSERTION              NC
    LED_PANIC                  Flashing
    -------------------------- ---------

  If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
  control of the application.  The following interfaces are then available
  for application control of the LEDs:

    uint32_t board_userled_initialize(void);
    void board_userled(int led, bool ledon);
    void board_userled_all(uint32_t ledset);

  Pushbuttons
  -----------
  To be provided

Serial Console
==============

The LPC4370-Link2 does not have RS-232 drivers or serial connectors on board.
USART, USART2 and USART3 are available on J12 as follows:

  ------ ------ -----------------------
  SIGNAL J12 PIN  LPC4370FET256 PIN
                  (TFBGA256 package)
  ------ ------ -----------------------
  U0_TXD pin  3  F6  P9_5  U0_TXD=Alt 4
  U0_RXD pin  4  F9  P9_6  U0_RXD=Alt 4
  U2_TXD pin  5  H8  P1_13 U1_TXD=Alt 1
  U2_RXD pin  6  J8  P1_14 U1_RXD=Alt 1
  U3_TXD pin  7  H8  P1_13 U1_TXD=Alt 1
  U3_RXD pin  8  J8  P1_14 U1_RXD=Alt 1
  ------ ------ -----------------------

  GND  is available on J12 pins 29 and 30
  5V   is available on J12 pin 2
  3.3v id available on J12 pin 1

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

LPC4370-Link2 Configuration Options
=====================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc43xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC4370=y

    CONFIG_ARCH_BOARD - Identifies the boards/ subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=LPC4370-Link2 (for the LPC4370-Link2 board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPC4370Link2=y

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
    CONFIG_LPC43_AUTONEG - Enable auto-negotiation

    CONFIG_LPC17_40_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
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
      Enable lpc17xx/lpc40xx-specific DMA support
    CONFIG_LPC43_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC43_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

Configurations
==============

Each LPC4370-Link2 configuration is maintained in a sub-directory and can be selected
as follow:

    tools/configure.sh LPC4370-Link2:<subdir>

Where <subdir> is one of the following:

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

    2. The project can exucute directly from SRAM with NuttX loaded by a debugger
       by setting the following configuration options.

         CONFIG_LPC43_BOOT_SRAM=y            : Executing in SRAM
         CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU EABI toolchain for Windows

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

   6. By default the LPC4370-Link2 port is configured to run from the onboard flash
      bank A at 0x1a000000. In order to achieve this, the resulting NuttX binary
      will need to have a checksum computed over the vector table and then be
      converted to a hex file which can then be flashed using a debugger such as
      the ULINK2 through Keil.

      The checksum can be computed using the checksum binary provided with the
      LPCXpresso IDE software suite as follows:

      ./checksum nuttx.bin -p LPC4370 -v

      This will modify the binary file, appending the checksum to the correct place
      at the end of the vector table.

      The binary must now be converted to a hex file, which can be achieved using
      the srec_cat utility, which is part of the SRecord package (srecord.sourceforge.net)
      as follows:

      srec_cat nuttx.bin -binary -offset 0x1a000000 -o nuttx.hex -intel --line-length=44

      Now the hex file can be loaded using a debugger, and the code will execute from
      flash.

STATUS
======

  1. This configuration derives from the LPC4330 Xplorer configuration.  In
     many cases there have been global substitutions for naming to the
     LPC4370 Link2 without corresponding updates to the technical description.
     Thus all technical details should be taken with a grain of salt.  GPIO
     definitions may actually are remnants of the LPC4330-Xplorer that still
     need clean-up.
