pirelli_dpl10
=============

  This directory contains the board support for Pirelli "Discus" DP-L10
  phones.

Contents
========

  * History
  * Hardware
  * Osmocom-BB Dependencies and Sercomm
  * Loading NuttX
  * Memory Map
  * USB Serial Console
  * NuttX OABI "buildroot" Toolchain
  * Generic OABI Toolchain
  * Configurations

History
=======
  This port is a variant of the compal_e88 configuration with the small
  change of enabling the IrDA serial console:

    - CONFIG_SERIAL_IRDA_CONSOLE=y

  This port is based on patches contributed by Denis Carikli for both the
  compal e99 and e88. At the time of initial check-in, the following phones
  were tested:

    - Pirelli DPL-10 nsh_highram loaded via romload in osmocon

  The patches were made by Alan Carvalho de Assis and Denis Carikli using
  the Stefan Richter's patches that can be found here:

    http://cgit.osmocom.org/cgit/nuttx-bb/log/?h=lputt%2Ftesting

Hardware
========

    * CPU/DBB: TI Calypso (D751992AZHH)

      See http://bb.osmocom.org/trac/wiki/Hardware/Calypso

    * ABB: TI Iota (TWL3014)

      Analog baseband chip.  See http://bb.osmocom.org/trac/wiki/Iota

    * GSM Transceiver: TI Rita (TRF6151)

      GSM Transceiver.  See http://bb.osmocom.org/trac/wiki/Rita

    * PA: SKY77328-13

      Quad-band GSM/GPRS:  See http://www.skyworksinc.com/Product.aspx?ProductID=434

    * Flash/SRAM: Spansion S71PL129NC0 128MBit/64MBit

      Combined FLASH and SDRAM:
      FLASH: 128Mbit
      SDRAM: 64Mbit

    * Wifi: Marvell 88W8385 802.11 MAC
            Marvell 88W8015 802.11b/g transceiver

    * Winbond W56940 ringtone chip

    * Sunplus SPCA552E multimedia controller

      Multimedia processor: integrates CMOS sensor interface, DSC processor, JPEG
      codec engine, LCM interface and other peripherals.

      I have not yet been able to find a data sheet for this chip.  I believe that
      it will be critical to develop drivers for the display.

    * LSI-65194A1 ASIC (seems to be a DSP for VoIP en-/decoding)

    * Silabs CP2102 USB UART (connected to UART_IRDA of the Calypso)

Osmocom-BB Dependencies and Sercomm
===================================

  Sercomm is an HDLC protocol used to communicate between a Calypso phone
  and the host PC.  By default, NuttX will not use sercomm (HDLC protocol) to
  communicate with the host system.  Sercomm is the transport used by
  osmocom-bb that runs on top of serial.  See
  http://bb.osmocom.org/trac/wiki/nuttx-bb/run for detailed the usage of nuttx
  with sercomm.

  If you build with sercomm, you must add support for sercomm in your
  configuration (CONFIG_SERCOMM_CONSOLE=y).  In this case, the build
  environment assumes that you have the osmocom-bb project directory at same
  level as the nuttx project:

    |- nuttx
    |- apps
    `- osmocom-bb

  If you attempt to build a sercomm-enaled configuration without osmocom-bb,
  you will get compilation errors in drivers/sercomm due to header files that
  are needed from the osmocom-bb directory.

Loading NuttX
=============

  General
  -------
  The osmocom-bb wiki describes how to load NuttX.  See
  http://bb.osmocom.org/trac/wiki/nuttx-bb for detailed information.
  The way that nuttx is loaded depends on the configuration (highram/compalram)
  and phone:

  - compalram is for the ramloader(for phone having a bootloader on flash)
  - highram is for phones having the romloader(if the phone has a bootrom)
    or for loading in the ram trough a special loader(loaded first on ram
    by talking to the ramloader) when having a ramloader(which can only
    load 64k).

  The Pirelli USB Serial Interface
  --------------------------------
  The Pirelli phone is epecially easy to use because you just use the
  supplied USB cable.  The phone already has an integrated Silabs CP210x
  USB-UART, which is supported by Linux.  No need for a T191 cable.

  Most of the phones seem to use USB vid:pid 0489:e003, which is mainline
  since Linux 2.6.36. You can do the following for Kernels < 2.6.36:

    # modprobe -v cp210x
    # echo "0489 e003" > /sys/bus/usb-serial/drivers/cp210x/new_id

  Loading NuttX
  -------------
  Here's how I load NuttX into the phone:

  - Take out the battery
  - Plug in the USB adapter into the phone then the computer
  - Start osmocon like: osmocon -p /dev/ttyUSB0 -m romload nuttx.bin
  - Put the battery back in

  This works most of the time.  Sometimes I have to take out and put in
  the battery a few times or re-start the whole set of steps but it's
  generally quite reliable.

Memory Map
=========

  Internal SRAM and ROM
  ---------------------
  Calypso has 256KB of internal SRAM (0x800000-0x83ffff, although some of
  this is, I believe, actually ROM).  Only this internal SRAM is used by
  these configurations.  The internal SRAM is broken up into two logical
  banks.

    LRAM (rw) : ORIGIN = 0x00800000, LENGTH = 0x00020000
    HRAM (rw) : ORIGIN = 0x00820000, LENGTH = 0x00020000

  Code can be loaded by the CalypsoBootloader only into HRAM beginning at
  address 0x00820000 and, hence, is restricted to 128KB (including then
  non-loaded sections:  Uninitialized data and the NuttX heap).

  SDRAM and NOR FLASH
  -------------------
  SDRAM is provided  by a Flash/SRAM: Spansion S71PL129NC0 part that provices
  128MBit (16MB) of FLASH and 64MBit (8MB) of SDRAM.

  *  SDRAM

    The Pirelli DP-L10 has 8MB of SDRAM beginning at address 0x01000000.
    This DRAM appears to be initialized by the Pirelli ROM loader and is
    ready for use with no further initialization required.

  * NOR FLASH

    The 16MB FLASH is at address 0x00000000.

USB Serial Console
==================

  These configurations are set up to use the Calypso IrDA UART as the serial
  port.  On the Pirelli phone, this port connects to the built-in USB-serial
  adaptor so that that NuttX serial console will be available on your PC as
  a USB serial device.  You should see something this using 'dmesg' when you
  plug the Pirelli phone into a PC running Linux:

    usb 5-2: new full speed USB device number 3 using uhci_hcd
    usb 5-2: New USB device found, idVendor=0489, idProduct=e003
    usb 5-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
    usb 5-2: Product: DP-L10
    usb 5-2: Manufacturer: Silicon Labs
    usb 5-2: SerialNumber: 0001
    usbcore: registered new interface driver usbserial
    USB Serial support registered for generic
    usbcore: registered new interface driver usbserial_generic
    usbserial: USB Serial Driver core
    USB Serial support registered for cp210x
    cp210x 5-2:1.0: cp210x converter detected
    usb 5-2: reset full speed USB device number 3 using uhci_hcd
    usb 5-2: cp210x converter now attached to ttyUSB0
    usbcore: registered new interface driver cp210x
    cp210x: v0.09:Silicon Labs CP210x RS232 serial adaptor driver


  Before you use this port to communicate with Nuttx, make sure that osmocon is
  no longer running.  Then start a serial terminal such as minicom on your host
  PC.  Configure the serial terminal so that it uses:

    Port:  /dev/ttyUSB0
    Baud:  115,200 8N1

JTAG and Alternative Serial Console
===================================

JTAG
  All JTAG lines, as well as the second uart (UART_MODEM), go to the
  unpopulated connector next to the display connector.  NOTE:  You have
  to disassemble the phone to get to this connector.


  --- ---------------------------
  PIN SIGNAL
  --- ---------------------------
    1 Vcc
    2 RX_MODEM
    3 TESTRSTz (Iota)
    4 TDI
    5 TMS
    6 TCK
    7 TX_MODEM
    8 TDO
    9 N/C
   10 GND
   11 N/C
   12 N/C
  --- ---------------------------

JTAG Apapter:

  ------- ----------- --------------- --------------------------------------
  JTAG    20-PIN      DESCRIPTION     NOTES
  SIGNAL  CONNECTOR
  ------- ----------- --------------- --------------------------------------
   Vcc    1, 2        Vcc
   nTRST  3           Reset           Connect this pin to the (active
                                      low) reset input of the target MCU.
                                      Some JTAG adapters driver nTRST (high
                                      and low). Others can can configure
                                      nTRST as open collector (only drive
                                      low).
   GND    4, 6, 8,    Ground
          10, 12, 14,
          16, 20
   TDI    5           JTAG Test Data  Use 10K-100K Ohm pull-up resistor to
                      Input           VCC
   TMS    7           JTAG Test Mode  Use 10K-100K Ohm pull-up resistor to
                      Select          VCC
   TCK    9           Clock into the  Use 10K-100K Ohm pull-down resistor to
                      core            GND
   RTCK   11          Return clock    Some JTAG adapters have adaptive clocking
                                      using an RTCK signal.
   DBGSEL 11          Debug Select    Some devices have special pins that
                                      enable the JTAG interface. For example,
                                      on the NXP LPC2129 the signal RTCK must
                                      be driven low during RESET to enable the
                                      JTAG interface.
   TDO    13          JTAG Test Data  Use 10K-100K Ohm pull-up resistor to VCC
                      Output
   DBGRQ  17          N/C
   DGBACK 19          N/C
   ISP    ??          ISP             Most NXP MCU's have an ISP pin which
                                      (when pulled low) can be used to cause
                                      the MCU to enter a bootloader on reset.
                                      Use 10K-100K Ohm pull up resistor.
  ------- ----------- --------------- --------------------------------------

NuttX OABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the ARM GCC toolchain (if
  different from the default in your PATH variable).

  If you have no ARMtoolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh pirelli_dpl10/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/arm7tdmi-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Generic OABI Toolchain
======================

  The NuttX OABI toolchain is selected with:

    CONFIG_ARM_TOOLCHAIN_BUILDROOT=y
    CONFIG_ARM_OABI_TOOLCHAIN=y

  In most cases, OsmocomBB is built with a different OABI toolchain with a
  prefix of arm-elf-.  To use that toolchain, change the configuration as
  follows:

    CONFIG_ARM_TOOLCHAIN_GNU_OABI=y

Configurations
==============

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. This configuration enables the serial interface on IrDA UART which
       will appears as a USB serial device.

       CONFIG_SERIAL_IRDA_CONSOLE=y

    3. By default, this configuration uses the CodeSourcery toolchain
       for Windows and builds under Cygwin (or probably MSYS).  That
       can easily be reconfigured, of course.

       CONFIG_HOST_LINUX=y                     : Builds under Linux
       CONFIG_ARM_TOOLCHAIN_BUILDROOT=y        : NuttX buildroot OABI toolchain
       CONFIG_ARM_OABI_TOOLCHAIN=y

       You can switch to use the generic arm-elf- GCC toolchain by
       setting:

        CONFIG_ARM_TOOLCHAIN_GNU_OABI=y        : General arm-elf- toolchain

    4. Support for builtin applications is enabled.  A builtin 'poweroff'
       command is supported.
