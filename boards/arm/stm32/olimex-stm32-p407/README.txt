README
======

The NuttX configuration for the Olimex STM32-P407 is derives more or less
directly from the Olimex STM32-P207 board support.  The P207 and P407 seem
to share the same board design.  Other code comes from the STM3240G board
support (which has the same crystal and clocking) and from the STM32 F4
Discovery (which has the same STM32 part)

Contents
========

  o Board Support
  o microSD Card Interface
  o OTGFS Host
  o Protect Mode Build
  o Configurations

Board Support
=============

The following peripherals are available in this configuration.

 - LEDs:       Show the system status

 - Buttons:    TAMPER-button, WKUP-button, J1-Joystick (consists of RIGHT-,
               UP-, LEFT-, DOWN-, and CENTER-button).

 - ADC:        ADC1 samples the red trim potentiometer AN_TR
               Built in app 'adc' works.

 - USB-FS-OTG: There is a USB-A-connector (host) connected to the full
               speed STM32 OTG inputs.

 - USB-HS-OTG: The other connector (device) is connected to the high speed
               STM32 OTG inputs.

 - CAN:        Built in app 'can' works, but apart from that not really
               tested.

 - Ethernet:   Ping to other station on the network works.

 - microSD:    Not fully functional.  See below.

 - LCD:        Nokia 6610. This is similar the Nokia 6100 LCD used on other
               Olimex boards.  There is a driver for that LCD at
               Obsoleted/nuttx/drivers/lcd/nokia6100.c, however, it was removed
               because it was not properly integrated.  It uses a 9-bit SPI
               interface which is difficult to get working properly.

- External     Support is included for the onboard SRAM.  It uses SRAM
  SRAM:        settings from another board that might need to be tweaked.
               Difficult to test because the SRAM conflicts with both
               RS232 ports.

- Other:       Buzzer, Camera, Temperature sensor, audio have not been
               tested.

 If so, then it requires a 9-bit

microSD Card Interface
======================

  microSD Connector
  -----------------

    ----------------- ----------------- ------------------------
    SD/MMC CONNECTOR        BOARD        GPIO CONFIGURATION(s
    PIN SIGNAL             SIGNAL          (no remapping)
    --- ------------- ----------------- -------------------------
    1   DAT2/RES      SD_D2/USART3_TX/  PC10 GPIO_SDIO_D2
                      SPI3_SCK
    2   CD/DAT3/CS    SD_D3/USART3_RX/  PC11 GPIO_SDIO_D3
                      SPI3_MISO
    3   CMD/DI        SD_CMD            PD2  GPIO_SDIO_CMD
    4   VDD           N/A               N/A
    5   CLK/SCLK      SD_CLK/SPI3_MOSI  PC12 GPIO_SDIO_CK
    6   VSS           N/A               N/A
    7   DAT0/D0       SD_D0/DCMI_D2     PC8  GPIO_SDIO_D0
    8   DAT1/RES      SD_D1/DCMI_D3     PC9  GPIO_SDIO_D1
    --- ------------- ----------------- -------------------------

    NOTES:
    1. DAT4, DAT4, DAT6, and DAT7 not connected.
    2. There are no alternative pin selections.
    3. There is no card detect (CD) GPIO input so we will not
       sense if there is a card in the SD slot or not.  This will
       make usage very awkward.

  Configuration
  -------------

  Enabling SDIO-based MMC/SD support:

    System Type->STM32 Peripheral Support
      CONFIG_STM32_SDIO=y                      : Enable SDIO support
      CONFIG_STM32_DMA2=y                      : DMA2 is needed by the driver

    Device Drivers -> MMC/SD Driver Support
      CONFIG_MMCSD=y                           : Enable MMC/SD support
      CONFIG_MMSCD_NSLOTS=1                    : One slot per driver instance
      # CONFIG_MMCSD_HAVE_CARDDETECT is not set : No card-detect GPIO
      # CONFIG_MMCSD_MMCSUPPORT is not set     : Interferes with some SD cards
      # CONFIG_MMCSD_SPI is not set            : No SPI-based MMC/SD support
      CONFIG_MMCSD_SDIO=y                      : SDIO-based MMC/SD support
      CONFIG_MMCSD_MULTIBLOCK_DISABLE=y        : Disable to keep things simple
      CONFIG_SDIO_DMA=y                        : Use SDIO DMA
      # CONFIG_SDIO_BLOCKSETUP is not set      : (not implemented)

    Library Routines
      CONFIG_SCHED_WORKQUEUE=y                 : Driver needs work queue support

    Application Configuration -> NSH Library
      CONFIG_NSH_ARCHINIT=y                    : NSH board-initialization

    Using the SD card
    -----------------

    1. Since there is no CD GPIO pin, the firmware sill not know if there is
       a card in the SD slot or not.  It will assume that there is and attempt
       to mount the SD card on power-up.  If there is no SD card in the card
       slot, there will be a long delay during initialization as the firmware
       attempts to query the non-existent card, timeout, and retry.

    2. After booting, an SDIO device will appear as /dev/mmcsd0

    3. If you try mounting an SD card with nothing in the slot, the
       mount will fail:

         nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
         nsh: mount: mount failed: 19

    STATUS:
    -------
    2017-01-28:  There is no card communication.  All commands to the SD card timeout.

OTGFS Host
==========
  STM32 USB OTG FS Host Board Support
  -----------------------------------
  A USB-A-connector (host) is connected to the full speed STM32 inputs.  These
  are the pins supported by the STM32:

    PIN  SIGNAL      DIRECTION
    ---- ----------- ----------
    PA8  OTG_FS_SOF  SOF clock output
    PA9  OTG_FS_VBUS VBUS input for device, Driven by external regulator by
                     host (not an alternate function)
    PA10 OTG_FS_ID   OTG ID pin (only needed in Dual mode)
    PA11 OTG_FS_DM   D- I/O
    PA12 OTG_FS_DP   D+ I/O

  These are the signals available on-board:

    OTG_FS_VBUS     Used host VBUS sensing (device input only)
    OTG_FS_DM       Data minus
    OTG_FS_DP       Dta plus

  NOTE: PA10 is currently used for DCMI_D1.  The USB OTGFS host will
  configure this as the ID input.

  VBUS power is provided via an LM3526 and driven by USB_FS_VBUSON:

    USB_FS_VBUSON  PC2  power on output to LM3526 #ENA
    USB_FS_FAULT   PB10 overcurrent input from LM3526 FLAG_A.

  STM32 USB OTG FS Host Driver Configuration
  ------------------------------------------
  Pre-requisites

    CONFIG_USBDEV          - Enable USB device support
    CONFIG_USBHOST         - Enable USB host support
    CONFIG_STM32_OTGFS     - Enable the STM32 USB OTG FS block
    CONFIG_STM32_SYSCFG    - Needed
    CONFIG_SCHED_WORKQUEUE - Worker thread support is required

  STM32 Options:

    CONFIG_STM32_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
      Default 128 (512 bytes)
    CONFIG_STM32_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
      in 32-bit words.  Default 96 (384 bytes)
    CONFIG_STM32_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
      words.  Default 96 (384 bytes)
    CONFIG_STM32_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128
    CONFIG_STM32_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
      want to do that?
    CONFIG_STM32_USBHOST_REGDEBUG - Enable very low-level register access
      debug.  Depends on CONFIG_DEBUG_FEATURES.
    CONFIG_STM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
      packets. Depends on CONFIG_DEBUG_FEATURES.

   Olimex STM32 P407 Configuration:

     CONFIG_STM32F_OLIMEXP407_PRIO - Priority of the USB host watier
       thread (default 100).
     CONFIG_STM32_OLIMEXP407_STACKSIZE - Stacksize of the USB host
       waiter thread (default 1024)

  Class Driver Configuration
  --------------------------
  Individual class drivers have additional configuration requirements.  The
  USB mass storage class, for example, requires FAT file system support.

    CONFIG_USBHOST_MSC=y

    CONFIG_FS_FAT=y
    CONFIG_FAT_LCNAMES=y
    CONFIG_FAT_LFN=y
    CONFIG_FAT_MAXFNAME=32

  This will enable USB HID keyboard support:

    CONFIG_USBHOST_HIDKBD=y
    CONFIG_HIDKBD_BUFSIZE=64
    CONFIG_HIDKBD_DEFPRIO=50
    CONFIG_HIDKBD_POLLUSEC=100000
    CONFIG_HIDKBD_STACKSIZE=1024

  And this will enable the USB keyboard example:

    CONFIG_EXAMPLES_HIDKBD=y
    CONFIG_EXAMPLES_HIDKBD_DEFPRIO=50
    CONFIG_EXAMPLES_HIDKBD_DEVNAME="/dev/kbda"
    CONFIG_EXAMPLES_HIDKBD_STACKSIZE=1024

  STATUS: The MSC configurations seems fully functional.  The HIDKBD seems rather
  flaky.  Sometimes the LEDs become very bright (indicating that it is being
  swamped with interrupts).  Data input is not clean with apps/examples/hidkbd:
  There are missing characters and sometimes duplicated characters.  This implies
  some logic issues, probably in drivers/usbhost/usbhost_hidkbd.c, with polling
  and data filtering.

Protected Mode Build
====================

  The "protected" mode build uses the Cormtex-M4 MPU to separate the FLASH and
  SRAM into kernel-mode and user-mode regions.  The kernel mode regions are
  then protected from any errant or mischievous behavior from user-space
  applications.

  Common notes for all protected mode builds follow:

  1. It is recommends to use a special make command; not just 'make' but make
     with the following two arguments:

       make pass1 pass2

     In the normal case (just 'make'), make will attempt to build both user-
     and kernel-mode blobs more or less interleaved.  That actual works!
     However, for me it is very confusing so I prefer the above make command:
     Make the user-space binaries first (pass1), then make the kernel-space
     binaries (pass2)

  2. At the end of the build, there will be several files in the top-level
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

  3. Combining .hex files.  If you plan to use the .hex files with your
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
        file to produce a single combined hex file:

        $ cat nuttx.hex nuttx_user.hex >combined.hex

     Then use the combined.hex file with the to write the FLASH image. With
     GDB this would be:

       gdb> mon reset
       gdb> mon halt
       gdb> mon clrbp
       gdb> load combined.hex

     If you do this a lot, you will probably want to invest a little time
     to develop a tool to automate these steps.

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each Olimex STM32-P407 configuration is maintained in a sub-directory and can be
selected as follow:

    tools/configure.sh olimex-stm32-p407:<subdir>

Where <subdir> is one of the configuration sub-directories listed in the
following section.

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make oldconfig
  make

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Serial Output

       This configuraiont produces all of its test output on the serial
       console.  This configuration has USART3 enabled as a serial console.
       This is the connector labeled RS232_2.  This can easily be changed
       by reconfiguring with 'make menuconfig'.

    3. Toolchain

       By default, the host platform is set to be Linux using the NuttX
       buildroot toolchain. The host and/or toolchain selection can easily
       be changed with 'make menuconfig'.

    4. Note that CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG is enabled so
       that the JTAG connection is not disconnected by the idle loop.

Configuration sub-directories
-----------------------------

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

  dhtxx:

    Configuration added by Abdelatif Guettouche for testing the the DHTxx
    sensor.  This configuration expects this setup:

      DHTXX_PIN_OUTPUT   PG9
      DHTXX_PIN_INPUT    PG9

    The STM32 free-running timer is also required.

  hidkbd

     This is another NSH configuration that supports a USB HID Keyboard
     device and the HID keyboard example at apps/examples/hidkbd.

    STATUS:
      2018-10-07:  Not all keyboards will connect successfully. I have not
        looked into the details but it may be that those keyboards are not
        compatible with the driver (which only accepts "boot" keyboards).
        Also, when typing input into the HID keyboard, characters are often
        missing and sometimes duplicated.  This is like some issue with the
        read logic of drivers/usbhost_hidkbc.c.

  kelf:

    This is a protected mode version of the apps/examples/elf test of
    loadable ELF programs.  This version is unique because the ELF programs
    are loaded into user space.

    NOTES:

    1. See build recommendations and instructions for combining the .hex
       files under the section entitled "Protected Mode Build" above.

    2. Unlike other versions of apps/examples/elf configurations, the test
       ELF programs are not provided internally on a ROMFS or CROMFS file
       system.  This is due to the fact that those file systems are built in
       user space and there is no mechanism in the build system to easily
       get them into the kernel space.

       Instead, the programs must be copied to a USB FLASH drive from your
       host PC.  The programs can be found at apps/examples/elf/tests/romfs.
       All of those files should be copied to the USB FLASH drive.  The
       apps/example/elf will wait on power up until the USB FLASH drive
       has been inserted and initialized.

  kmodule:

    This is a protected mode version of the apps/examples/module test of
    loadable ELF kernel modules.  This version is unique because the ELF
    programs are loaded into the protected kernel space.

    NOTES:

    1. See build recommendations and instructions for combining the .hex
       files under the section entitled "Protected Mode Build" above.

    2. Unlike other versions of apps/examples/module configurations, the test
       ELF modules are not provided internally on a ROMFS or CROMFS file
       system.  This is due to the fact that those file systems are built in
       user space and there is no mechanism in the build system to easily
       get them into the kernel space.

       Instead, the module(s) must be copied to a USB FLASH drive from your
       host PC.  The module(s) can be found at apps/examples/module/driver/fsroot.
       All of those file(s) should be copied to the USB FLASH drive.  Like the
       kelf configuration, the logic in apps/example/module will wait on power
       up until the USB FLASH drive has been inserted and initialized.

    STATUS:
      2018-08-07:  After some struggle, the configuration appears to be
        working correctly.

  knsh:

    This is identical to the nsh configuration below except that NuttX
    is built as a PROTECTED mode, monolithic module and the user applications
    are built separately.

    NOTES:

    1. See build recommendations and instructions for combining the .hex
       files under the section entitled "Protected Mode Build" above.

  module:

    A simple stripped down NSH configuration that was used for testing NuttX
    OS modules using the test at apps/examples/module.  Key difference from
    the nsh configuration include these additions to the configuration file:

      CONFIG_BOARDCTL_OS_SYMTAB=y
      CONFIG_EXAMPLES_MODULE=y
      CONFIG_EXAMPLES_MODULE_BUILTINFS=y
      CONFIG_EXAMPLES_MODULE_DEVMINOR=0
      CONFIG_EXAMPLES_MODULE_DEVPATH="/dev/ram0"
      CONFIG_FS_ROMFS=y
      CONFIG_LIBC_ARCH_ELF=y
      CONFIG_MODULE=y
      CONFIG_LIBC_MODLIB=y
      CONFIG_MODLIB_MAXDEPEND=2
      CONFIG_MODLIB_ALIGN_LOG2=2
      CONFIG_MODLIB_BUFFERSIZE=128
      CONFIG_MODLIB_BUFFERINCR=32

     The could be followed may be added for testing shared libraries in the
     FLAT build using apps/examples/sotest (assuming that you also have SD
     card support enabled and that the SD card is mount at /mnt/sdcard):

      CONFIG_LIBC_DLFCN=y
      CONFIG_EXAMPLES_SOTEST=y
      CONFIG_EXAMPLES_SOTEST_BINDIR="/mnt/sdcard"

    NOTE: You must always have:

      CONFIG_STM32_CCMEXCLUDE=y

    because code cannot be executed from CCM memory.

    STATUS:
    2018-06-01: Configuration added.  Works perfectly.

  nsh:

    This is the NuttShell (NSH) using the NSH startup logic at
    apps/examples/nsh

    NOTES:

    1. USB host support for USB FLASH sticks is enabled.  See the notes
       above under "OTGFS Host".

       STATUS: I have seen this work with some FLASH sticks but not with
       others.  I have not studied the failure case carefully.  They seem
       to fail because the request is NAKed.  That is not a failure, however,
       that is normal behavior when the FLASH is not ready.

       There have been other cases like this with the STM32 host drivers:
       in the event of NAKs, other drivers retry and wait for the data.  The
       STM32 does not but returns the NAK failure immediately.  My guess is
       that there needs to be be some retry logic to the driver 100%
       reliable.

    2. Kernel Modules / Shared Libraries

       I used this configuration for testing NuttX kernel modules in the
       FLAT build with the following configuration additions to the
       configuration file:

         CONFIG_BOARDCTL_OS_SYMTAB=y
         CONFIG_EXAMPLES_MODULE=y
         CONFIG_EXAMPLES_MODULE_BUILTINFS=y
         CONFIG_EXAMPLES_MODULE_DEVMINOR=0
         CONFIG_EXAMPLES_MODULE_DEVPATH="/dev/ram0"
         CONFIG_FS_ROMFS=y
         CONFIG_LIBC_ARCH_ELF=y
         CONFIG_MODULE=y
         CONFIG_LIBC_MODLIB=y
         CONFIG_MODLIB_ALIGN_LOG2=2
         CONFIG_MODLIB_BUFFERINCR=32
         CONFIG_MODLIB_BUFFERSIZE=128

       Add the following for testing shared libraries in the FLAT
       build:

         CONFIG_LIBC_DLFCN=y
         CONFIG_EXAMPLES_SOTEST=y
         CONFIG_EXAMPLES_SOTEST_BUILTINFS=y
         CONFIG_EXAMPLES_SOTEST_DEVMINOR=1
         CONFIG_EXAMPLES_SOTEST_DEVPATH="/dev/ram1"

  zmodem:

    This configuration was used to test the zmodem utilities at
    apps/system/zmodem.  Two serial ports are used in this configuration:

      1. USART6 (RS232_1) is the serial console (because it does not support
         hardware flow control). It is configured 115200 8N1.
      2. USART3 (RS232_2) is the zmodem port and does require that hardware
         flow control be enabled for use.  It is configured 9600 8N1.

    On the target these will correspond to /dev/ttyS0 and /dev/ttyS1,
    respectively.

    It is possible to configure a system without hardware flow control and
    using the same USART for both the serial console and for zmodem.
    However, you would have to take extreme care with buffering and data
    throughput considerations to assure that there is no Rx data overrun.

    General usage instructions:

    1. Common Setup

      [on target]
      nsh> mount -t vfat /dev/sda /mnt

      [on Linux host]
      $ sudo stty -F /dev/ttyS0 9600
      $ sudo stty -F /dev/ttyS0 crtscts *
      $ sudo stty -F /dev/ttyS0 raw
      $ sudo stty -F /dev/ttyS0

      * Because hardware flow control will be enabled on the target side
        in this configuration.

    2. Host-to-Target File Transfer

      [on target]
      nsh> rz

      [on host]
      $ sudo sz <filename> [-l nnnn] </dev/ttyS0 >/dev/ttyS0

    Where <filename> is the file that you want to transfer. If -l nnnn is
    not specified, then there will likely be packet buffer overflow errors.
    nnnn should be set to a strictly less than CONFIG_SYSTEM_ZMODEM_PKTBUFSIZE.
    All testing was performed with -l 512.

    If you are using the NuttX implementation of rz and sz on the Linux host,
    then the last command simplifies to just:

      [on host]
      $ cp README.txt /tmp/.
      $ sudo ./sz -d /dev/ttyS1 README.txt

    Assuming that /dev/ttyS0 is the serial and /dev/ttyS1 is the zmodem port
    on the Linux host as well.  NOTE:  By default, files will be transferred
    to and from the /tmp directory only.

    Refer to the README file at apps/examples/zmodem for detailed information
    about building rz/sz for the host and about zmodem usage in general.

    3. Target-to-Host File Transfer

      [on host]
      $ rz </dev/ttyS0 >/dev/ttyS0

    The transferred file will end up in the current directory.

    If you are using the NuttX implementation of rz and sz on the Linux host,
    then the last command simplifies to just:

      [on host]
      $ ./rz

    The transferred file will lie in the /tmp directory.

    Then on the target side:

      [on target]
      nsh sz <filename>

    Where <filename> is the file that you want to transfer.

STATUS
======

2016-12-21: This board configuration was ported from the Olimex STM32 P207
  port.  Note that none of the above features have been verified.  USB, CAN,
  ADC, and Ethernet are disabled in the base NSH configuration until they
  can be verified.  These features should be functional but may required
  some tweaks due to the different clock configurations.  The Olimex STM32
  P207 nsh/defconfig would be a good starting place for restoring these
  feature configurations.

  CCM memory is not included in the heap (CONFIG_STM32_CCMEXCLUDE=y) because
  it does not support DMA, leaving only 128KiB for program usage.

2017-01-23:  Added the knsh configuration and support for the PROTECTED
  build mode.

2018-05-27:  Added the zmodem configuration.  Verified correct operation
  with host-to-target transfers (using Linux sz command).  There appears
  to be a problem using the NuttX sz command running on the host???

2018-05-28:  Verified correct operation with target-to-host transfers (using
  Linux rz command).  There appears to be a problem using the NuttX rz
  command running on the host???

2018-06-01: Added the module configuration.  Works perfectly.
