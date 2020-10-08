README for the Espressif ESP32 Core board (V2)
==============================================

  The ESP32 is a dual-core system from Espressif with two Harvard
  architecture Xtensa LX6 CPUs. All embedded memory, external memory and
  peripherals are located on the data bus and/or the instruction bus of
  these CPUs. With some minor exceptions, the address mapping of two CPUs
  is symmetric, meaning they use the same addresses to access the same
  memory. Multiple peripherals in the system can access embedded memory via
  DMA.

  The two CPUs are named "PRO_CPU" and "APP_CPU" (for "protocol" and
  "application"), however for most purposes the two CPUs are
  interchangeable.

Contents
========

  o ESP32 Features
  o ESP32 Toolchain
  o Memory Map
  o Serial Console
  o Buttons and LEDs
  o Ethernet
  o SMP
  o OpenOCD for the ESP32
  o Executing and Debugging from FLASH and IRAM
  o Configurations
  o Things to Do

STATUS
======

  Currently we have support to UART, SPI, I2C, Ethernet, etc.

  Espressif is working to include support to WiFi and Bluetooth, but
  it will depends on their external libraries to get it up and running.

ESP32 Features
==============

  * Address Space
    - Symmetric address mapping
    - 4 GB (32-bit) address space for both data bus and instruction bus
    - 1296 KB embedded memory address space
    - 19704 KB external memory address space
    - 512 KB peripheral address space
    - Some embedded and external memory regions can be accessed by either
      data bus or instruction bus
    - 328 KB DMA address space
  * Embedded Memory
    - 448 KB Internal ROM
    - 520 KB Internal SRAM
    - 8 KB RTC FAST Memory
    - 8 KB RTC SLOW Memory
  * External Memory
    Off-chip SPI memory can be mapped into the available address space as
    external memory. Parts of the embedded memory can be used as transparent
    cache for this external memory.
    - Supports up to 16 MB off-Chip SPI Flash.
    - Supports up to 8 MB off-Chip SPI SRAM.
  * Peripherals
    - 41 peripherals
  * DMA
    - 13 modules are capable of DMA operation

ESP32 Toolchain
===============

  You must use the custom Xtensa toolchain in order to build the ESP32 Core
  BSP.  The steps to build toolchain with crosstool-NG on Linux are as
  follows:

    git clone https://github.com/espressif/crosstool-NG.git
    cd crosstool-NG
    git checkout esp-2019r2
    git submodule update --init

    ./bootstrap && ./configure --enable-local && make

    ./ct-ng xtensa-esp32-elf
    ./ct-ng build

    chmod -R u+w builds/xtensa-esp32-elf

    export PATH="crosstool-NG/builds/xtensa-esp32-elf/bin:$PATH"

  These steps are given in setup guide in ESP-IDF repository:
  https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup-scratch.html

Memory Map
==========

  Address Mapping
  ----------- ---------- ---------- --------------- ---------------
  BUS TYPE    START      LAST       DESCRIPTION     NOTES
  ----------- ---------- ---------- --------------- ---------------
              0x00000000 0x3F3FFFFF                 Reserved
  Data        0x3F400000 0x3F7FFFFF External Memory
  Data        0x3F800000 0x3FBFFFFF External Memory
              0x3FC00000 0x3FEFFFFF                 Reserved
  Data        0x3FF00000 0x3FF7FFFF Peripheral
  Data        0x3FF80000 0x3FFFFFFF Embedded Memory
  Instruction 0x40000000 0x400C1FFF Embedded Memory
  Instruction 0x400C2000 0x40BFFFFF External Memory
              0x40C00000 0x4FFFFFFF                 Reserved
  Data /      0x50000000 0x50001FFF Embedded Memory
  Instruction
              0x50002000 0xFFFFFFFF                 Reserved

  Embedded Memory
  ----------- ---------- ---------- --------------- ---------------
  BUS TYPE    START      LAST       DESCRIPTION     NOTES
  ----------- ---------- ---------- --------------- ---------------
  Data        0x3ff80000 0x3ff81fff RTC FAST Memory PRO_CPU Only
              0x3ff82000 0x3ff8ffff                 Reserved
  Data        0x3ff90000 0x3ff9ffff Internal ROM 1
              0x3ffa0000 0x3ffadfff                 Reserved
  Data        0x3ffae000 0x3ffdffff Internal SRAM 2 DMA
  Data        0x3ffe0000 0x3fffffff Internal SRAM 1 DMA

  Boundary Address
  ----------- ---------- ---------- --------------- ---------------
  BUS TYPE    START      LAST       DESCRIPTION     NOTES
  ----------- ---------- ---------- --------------- ---------------
  Instruction 0x40000000 0x40007fff Internal ROM 0  Remap
  Instruction 0x40008000 0x4005ffff Internal ROM 0
              0x40060000 0x4006ffff                 Reserved
  Instruction 0x40070000 0x4007ffff Internal SRAM 0 Cache
  Instruction 0x40080000 0x4009ffff Internal SRAM 0
  Instruction 0x400a0000 0x400affff Internal SRAM 1
  Instruction 0x400b0000 0x400b7FFF Internal SRAM 1 Remap
  Instruction 0x400b8000 0x400bffff Internal SRAM 1
  Instruction 0x400c0000 0x400c1FFF RTC FAST Memory PRO_CPU Only
  Data /      0x50000000 0x50001fff RTC SLOW Memory
  Instruction

  External Memory
  ----------- ---------- ---------- --------------- ---------------
  BUS TYPE    START      LAST       DESCRIPTION     NOTES
  ----------- ---------- ---------- --------------- ---------------
  Data        0x3f400000 0x3f7fffff External Flash  Read
  Data        0x3f800000 0x3fbfffff External SRAM   Read and Write

  Boundary Address
  ----------------
  Instruction 0x400c2000 0x40bfffff 11512 KB External Flash Read

  Linker Segments
  ------------------ ---------- ---------- ---- ----------------------------
  DESCRIPTION        START      END        ATTR LINKER SEGMENT NAME
  ------------------ ---------- ---------- ---- ----------------------------
  FLASH mapped data: 0x3f400010 0x3fc00010  R   drom0_0_seg
    - .rodata
    - Constructors/destructors
  COMMON data RAM:   0x3ffb0000 0x40000000  RW  dram0_0_seg  (NOTE 1,2)
    - .bss/.data
  IRAM for PRO cpu:  0x40080000 0x400a0000  RX  iram0_0_seg
    - Interrupt Vectors
    - Low level handlers
    - Xtensa/Espressif libraries
  RTC fast memory:   0x400c0000 0x400c2000  RWX rtc_iram_seg (PRO_CPU only)
    - .rtc.text (unused?)
  FLASH:             0x400d0018 0x40400018  RX  iram0_2_seg  (actually FLASH)
    - .text
  RTC slow memory:   0x50000000 0x50001000  RW  rtc_slow_seg (NOTE 3)
    - .rtc.data/rodata (unused?)

  NOTE 1: Linker script will reserve space at the beginning of the segment
          for BT and at the end for trace memory.
  NOTE 2: Heap enads at the top of dram_0_seg
  NOTE 3: Linker script will reserve space at the beginning of the segment
          for co-processor reserve memory and at the end for ULP coprocessor
          reserve memory.

Serial Console
==============

  UART0 is, by default, the serial console.  It connects to the on-board
  CP2102 converter and is available on the USB connector USB CON8 (J1).

  It will show up as /dev/ttypUSB[n] where [n] will probably be 0 (is it 1
  on my PC because I have a another device at ttyUSB0).

Buttons and LEDs
================

  Buttons
  -------
  There are two buttons labeled Boot and EN.  The EN button is not available
  to software.  It pulls the chip enable line that doubles as a reset line.

  The BOOT button is connected to IO0.  On reset it is used as a strapping
  pin to determine whether the chip boots normally or into the serial
  bootloader.  After reset, however, the BOOT button can be used for software
  input.

  LEDs
  ----
  There are several on-board LEDs for that indicate the presence of power
  and USB activity.  None of these are available for use by software.

Ethernet
========

  ESP32 has a 802.11 hardware MAC, so just connects to external PHY chip.
  Due to ESP32's GPIOs are not enough, so recommanded users to use RMII
  to connect ESP32 to PHY chip, current driver also only supports RMII option.

  The RMII GPIO pins are fixed, but the SMI and functional GPIO pins are optional.
  
  RMII GPIO pins are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO25       <-->       RXD[0]
        IO26       <-->       RXD[1]
        IO27       <-->       CRS_DV
        IO0        <-->       REF_CLK
        IO19       <-->       TXD[0]
        IO21       <-->       TX_EN
        IO22       <-->       TXD[1]

  SMI GPIO pins (default option) are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO18       <-->       MDIO
        IO23       <-->       MDC

  Functional GPIO pins(default option) are as following:

      ESP32 GPIO          PHY Chip GPIO
        IO5        <-->      Reset_N

Espressif has an offcial Ethernet development board:

  https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-ethernet-kit.html

This driver has been tested according to this board and ESP32 core
board + LAN8720 module. If users have some issue about using this driver,
please refer the upper official document, specially the issue that GPIO0
causes failing to bring the ESP32 chip up.

SMP
===

  The ESP32 has 2 CPUs.  Support is included for testing an SMP configuration.
  That configuration is still not yet ready for usage but can be enabled with
  the following configuration settings:

    RTOS Features -> Tasks and Scheduling
      CONFIG_SPINLOCK=y
      CONFIG_SMP=y
      CONFIG_SMP_NCPUS=2
      CONFIG_SMP_IDLETHREAD_STACKSIZE=3072

  Debug Tip:  During debug session, OpenOCD may mysteriously switch from one
  CPU to another.  This behavior can be eliminated by uncommenting one of the
  following in scripts/esp32.cfg

    # Only configure the PRO CPU
    #set ESP32_ONLYCPU 1
    # Only configure the APP CPU
    #set ESP32_ONLYCPU 2

  Open Issues:

  1. Cache Issues.  I have not thought about this yet, but certainly caching is
     an issue in an SMP system:

     - Cache coherency.  Are there separate caches for each CPU?  Or a single
       shared cache?  If the are separate then keep the caches coherent will
       be an issue.
     - Caching MAY interfere with spinlocks as they are currently implemented.
       Waiting on a cached copy of the spinlock may result in a hang or a
       failure to wait.

  2. Assertions.  On a fatal assertions, other CPUs need to be stopped.

OpenOCD for the ESP32
=====================

  First you in need some debug environment which would be a JTAG emulator
  and the ESP32 OpenOCD software which is available here:
  https://github.com/espressif/openocd-esp32

  OpenOCD Documentation
  ---------------------
  There is on overiew of the use of OpenOCD here:
  https://dl.espressif.com/doc/esp-idf/latest/openocd.html
  This document is also available in ESP-IDF source tree in docs
  directory (https://github.com/espressif/esp-idf).

  OpenOCD Configuration File
  --------------------------
  A template ESP32 OpenOCD configuration file is provided in
  ESP-IDF docs directory (esp32.cfg).  Since you are not using
  FreeRTOS, you will need to uncomment the line:

    set ESP32_RTOS none

  in the OpenOCD configuration file.  You will also need to change
  the source line from:

    find interface/ftdi/tumpa.cfg

  to reflect the physical JTAG adapter connected.

  NOTE: A copy of this OpenOCD configuration file available in the NuttX
  source tree at nuttx/boards/xtensa/esp32/esp32-core/scripts/esp32.cfg .  It has these
  modifications:

    - The referenced "set ESP32_RTOS none" line has been uncommented
    - The "find interface/ftdi/tumpa.cfg".  This means that you will
      need to specify the interface configuration file on the OpenOCD
      command line.

  NOTE: Another OpenOCD configuration file is available in the NuttX
  source tree at
  nuttx/boards/xtensa/esp32/esp32-core/scripts/esp32-ft232h.cfg .
  It has been tested with:

    - ESP32-DevKitC V4

      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/esp32/get-started-devkitc.html

    - Akizukidenshi's FT232HL, a FT232H based JTAG adapter

      http://akizukidenshi.com/catalog/g/gK-06503/

      With JP3 and JP4 closed, and connected to ESP32 as:

        ---------------- -------
        ESP32-DevKitC V4 FT232HL
        ---------------- -------
        J2       J3      J2
        -------- ------- -------
        IO13             AD0     (TCK)
        IO12             AD1     (TDI)
                 IO15    AD2     (TDO)
        IO14             AD3     (TMS)
        GND              GND
        -------- ------- -------

    - The following version of OpenOCD from ESP-IDF (macOS version)

      % openocd --version
      Open On-Chip Debugger  v0.10.0-esp32-20191114 (2019-11-14-14:19)
      Licensed under GNU GPL v2
      For bug reports, read
              http://openocd.org/doc/doxygen/bugs.html
      %

  General OpenOCD build instructions
  ----------------------------------
  Installing OpenOCD.  The sources for the ESP32-enabled variant of
  OpenOCD are available from Espressifs Github. To download the source,
  use the following commands:

    git clone https://github.com/espressif/openocd-esp32.git
    cd openocd-esp32
    git submodule init
    git submodule update

  Then look at the README and the docs/INSTALL.txt files in the
  openocd-esp32 directory for further instructions.  There area
  separate README files for Linux/Cygwin, macOS, and Windows.  Here
  is what I ended up doing (under Linux):

    cd openocd-esp32
    ./bootstrap
    ./configure
    make

   If you do not do the install step, then you will have a localhost
   version of the OpenOCD binary at openocd-esp32/src.

  Starting the OpenOCD Server
  ---------------------------

    - cd to openocd-esp32 directory
    - copy the modified esp32.cfg script to this directory

  Then start OpenOCD by executing a command like the following.  Here
  I assume that:

    - You did not install OpenOCD; binaries are available at
      openocd-esp32/src and interface scripts are in
      openocd-eps32/tcl/interface
    - I select the configuration for the Olimex ARM-USB-OCD
      debugger.

  Then the command to start OpenOCD is:

    sudo ./src/openocd -s ./tcl -f tcl/interface/ftdi/olimex-arm-usb-ocd.cfg -f ./esp32.cfg

  I then see:

    Open On-Chip Debugger 0.10.0-dev-g3098897 (2016-11-14-12:19)
    Licensed under GNU GPL v2
    For bug reports, read
            http://openocd.org/doc/doxygen/bugs.html
    adapter speed: 200 kHz
    force hard breakpoints
    Info : clock speed 200 kHz
    Info : JTAG tap: esp32.cpu0 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
    Info : JTAG tap: esp32.cpu1 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
    Info : esp32.cpu0: Debug controller was reset (pwrstat=0x5F, after clear 0x0F).
    Info : esp32.cpu0: Core was reset (pwrstat=0x5F, after clear 0x0F).

  Connecting a debugger to OpenOCD
  --------------------------------
  OpenOCD should now be ready to accept gdb connections. If you have
  compiled the ESP32 toolchain using Crosstool-NG, or if you have
  downloaded a precompiled toolchain from the Espressif website, you
  should already have xtensa-esp32-elf-gdb, a version of gdb that can
  be used for this

  First, make sure the project you want to debug is compiled and
  flashed into the ESP32’s SPI flash. Then, in a different console
  than OpenOCD is running in, invoke gdb. For example, for the
  template app, you would do this like such:

    cd nuttx
    xtensa-esp32-elf-gdb -ex 'target remote localhost:3333' nuttx

  This should give you a gdb prompt.

  Breakpoints
  -----------
  You can set up to 2 hardware breakpoints, which can be anywhere in the
  address space. Also 2 hardware watchpoints.

  The openocd esp32.cfg file currently forces gdb to use hardware
  breakpoints, I believe because software breakpoints (or, at least, the
  memory map for automatically choosing them) aren't implemented yet
  (as of 2016-11-14).

  JTAG Emulator
  -------------
  The documentation indicates that you need to use an external JTAG
  like the TIAO USB Multi-protocol Adapter and the Flyswatter2.
  The instructions at http://www.esp32.com/viewtopic.php?t=381 show
  use of an FTDI C232HM-DDHSL-0 USB 2.0 high speed to MPSSE cable.

  The ESP32 Core v2 board has no on board JTAG connector.  It will
  be necessary to make a cable or some other board to connect a JTAG
  emulator.  Refer to http://www.esp32.com/viewtopic.php?t=381 "How
  to debug ESP32 with JTAG / OpenOCD / GDB 1st part connect the
  hardware."

  Relevant pin-out:

    -------- ----------
    PIN      JTAG
    LABEL    FUNCTION
    -------- ----------
    IO14     TMS
    IO12     TDI
    GND      GND
    IO13     TCK
    -------- ----------
    IO15     TDO
    -------- ----------

  You can find the mapping of JTAG signals to ESP32 GPIO numbers in
  "ESP32 Pin List" document found here:
  http://espressif.com/en/support/download/documents?keys=&field_type_tid%5B%5D=13

  I put the ESP32 on a prototyping board and used a standard JTAG 20-pin
  connector with an older Olimex JTAG that I had.  Here is how I wired
  the 20-pin connector:

    ----------------- ----------
    20-PIN JTAG       ESP32 PIN
    CONNECTOR         LABEL
    ----------------- ----------
     1 VREF  INPUT    3V3
     3 nTRST OUTPUT   N/C
     5 TDI   OUTPUT   IO12
     7 TMS   OUTPUT   IO14
     9 TCLK  OUTPUT   IO13
    11 RTCK  INPUT    N/C
    13 TDO   INPUT    IO15
    15 RESET I/O      N/C
    17 DBGRQ OUTPUT   N/C
    19 5V    OUTPUT   N/C
    ------------ ----------
     2 VCC   INPUT    3V3
     4 GND   N/A      GND
     6 GND   N/A      GND
     8 GND   N/A      GND
    10 GND   N/A      GND
    12 GND   N/A      GND
    14 GND   N/A      GND
    16 GND   N/A      GND
    18 GND   N/A      GND
    20 GND   N/A      GND
    ------------ ----------

 Executing and Debugging from FLASH and IRAM
 ===========================================

  Enable Debug Symbols
  --------------------
  To debug with GDB, you will need to enable symbols in the build.  You do this
  with 'make menuconfig' then selecting:

    - "Build Setup" -> "Debug Options" -> "Generate Debug Symbols"

  And, to make debugging easier, also disable optimizations.  This will make
  your code a lot bigger:

    - "Build Setup" -> "Optimization Level" -> "Suppress Optimization"

  FLASH
  -----
  OpenOCD currently doesn't have a FLASH driver for ESP32, so you can load
  code into IRAM only via JTAG. FLASH-resident sections like .FLASH.rodata
  will fail to load.  The bootloader in ROM doesn't parse ELF, so any image
  which is bootloaded from FLASH has to be converted into a custom image
  format first.

  The tool esp-idf uses for flashing is a command line Python tool called
  "esptool.py" which talks to a serial bootloader in ROM.  A version is
  supplied in the esp-idf codebase in components/esptool_py/esptool, the
  "upstream" for that tool is here and now supports ESP32.

    https://github.com/espressif/esptool/

  To FLASH an ELF via the command line is a two step process, something like
  this:

    esptool.py --chip esp32 elf2image --flash_mode dio --flash_size 4MB -o nuttx.bin nuttx
    esptool.py --chip esp32 --port COMx write_flash 0x1000 bootloader.bin 0x8000 partition_table.bin 0x10000 nuttx.bin

  The first step converts an ELF image into an ESP32-compatible binary
  image format, and the second step flashes it (along with bootloader image and
  partition table binary.)
  The offset for the partition table may vary, depending on ESP-IDF
  configuration, CONFIG_PARTITION_TABLE_OFFSET, which is by default 0x8000
  as of writing this.

  To put the ESP32 into serial flashing mode, it needs to be reset with IO0 held
  low.  On the Core boards this can be accomplished by holding the button marked
  "Boot" and pressing then releasing the button marked "EN".  Actually, esptool.py
  can enter bootloader mode automatically (via RTS/DTR control lines), but
  unfortunately a timing interaction between the Windows CP2012 driver and the
  hardware means this doesn't currently work on Windows.

  Secondary Boot Loader / Partition Table
  ---------------------------------------
  See https://github.com/espressif/esp-idf/tree/master/components/bootloader
  and https://github.com/espressif/esp-idf/tree/master/components/partition_table .

  The secondary boot loader by default programs a RTC watchdog timer.
  As NuttX doesn't know the timer, it reboots every ~9 seconds. You can
  disable the timer by tweaking sdkconfig CONFIG_BOOTLOADER_WDT_ENABLE
  and rebuild the boot loader.

  Running from IRAM with OpenOCD
  ------------------------------
  Running from IRAM is a good debug option.  You should be able to load the
  ELF directly via JTAG in this case, and you may not need the bootloader.

  NuttX supports a configuration option, CONFIG_ESP32CORE_RUN_IRAM, that may be
  selected for execution from IRAM.  This option simply selects the correct
  linker script for IRAM execution.

  Skipping the Secondary Bootloader
  ---------------------------------
  It is possible to skip the secondary bootloader and run out of IRAM using
  only the primary bootloader if your application of small enough (< 128KiB code,
  <180KiB data), then you can simplify initial bring-up by avoiding second stage
  bootloader. Your application will be loaded into IRAM using first stage
  bootloader present in ESP32 ROM. To achieve this, you need two things:

    1. Have a linker script which places all code into IRAM and all data into
       IRAM/DRAM

    2. Use "esptool.py" utility found in ESP-IDF to convert application .elf
       file into binary format which can be loaded by first stage bootloader.

  Again you would need to link the ELF file and convert it to binary format suitable
  for flashing into the board.  The command should to convert ELF file to binary
  image looks as follows:

    python esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 elf2image --flash_mode "dio" --flash_freq "40m" --flash_size "2MB" -o nuttx.bin nuttx

  To flash binary image to your development board, use the same esptool.py utility:

    python esp-idf/components/esptool_py/esptool/esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash -z --flash_mode dio --flash_freq 40m --flash_size 2MB 0x1000 nuttx.bin

  The argument before app.bin (0x1000) indicates the offset in flash where binary
  will be written. ROM bootloader expects to find an application (or second stage
  bootloader) image at offset 0x1000, so we are writing the binary there.

  Clocking
  --------
  Right now, the NuttX port depends on the bootloader to initialize hardware,
  including basic (slow) clocking.    If I had the clock configuration logic,
  would I be able to run directly out of IRAM without a bootloader?  That
  might be a simpler bring-up.

  Sample OpenOCD Debug Steps
  --------------------------
  I did the initial bring-up using the IRAM configuration and OpenOCD.  Here
  is a synopsis of my debug steps:

  boards/xtensa/esp32/esp32-core/configs/nsh with

    CONFIG_DEBUG_ASSERTIONS=y
    CONFIG_DEBUG_FEATURES=y
    CONFIG_DEBUG_SYMBOLS=y
    CONFIG_ESP32CORE_RUN_IRAM=y

  I also made this change configuration which will eliminate all attempts to
  re-configure serial. It will just use the serial settings as they were left
  by the bootloader:

    CONFIG_SUPPRESS_UART_CONFIG=y

  Start OpenOCD:

    cd ../openocde-esp32
    cp ../nuttx/boards/xtensa/esp32/esp32-core/scripts/esp32.cfg .
    sudo ./src/openocd -s ./tcl/ -f tcl/interface/ftdi/olimex-arm-usb-ocd.cfg -f ./esp32.cfg

  Start GDB and load code:

    cd ../nuttx
    xtensa-esp32-elf-gdb -ex 'target remote localhost:3333' nuttx
    (gdb) load nuttx
    (gdb) mon reg pc [value report by load for entry point]
    (gdb) s

  Single stepping works fine for me as do breakpoints:

    Breakpoint 1, up_timer_initialize () at chip/esp32_timerisr.c:172
    72 {
    (gdb) n
    esp32.cpu0: Target halted, pc=0x400835BF
    187 g_tick_divisor = divisor;
    (gdb) ...

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each ESP32 core configuration is maintained in sub-directories and
  can be selected as follow:

    tools/configure.sh esp32-core:<subdir>
    make oldconfig

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  If this is a Windows native build, then configure.bat should be used
  instead of configure.sh:

    configure.bat esp32-core\<subdir>

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply,
  nuttx.

    make oldconfig
    make

  The <subdir> that is provided above as an argument to the
  tools/configure.sh must be is one of the directories listed below.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART0 (see the "Serial Console" section above).

  3. By default, these configurations assume a 40MHz crystal on-
     board:

     CONFIG_ESP32CORE_XTAL_40MZ=y
     # CONFIG_ESP32CORE_XTAL_26MHz is not set

  4. Default configurations are set to run from FLASH.  You will need
     to set CONFIG_ESP32CORE_RUN_IRAM=y for now (see the " Executing
     and Debugging from FLASH and IRAM" section above).

     To select this option, do 'make menuconfig'.  Then you can find
     the selection under the "Board Selection" menu as "Run from IRAM".

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:

    1. Uses the CP2102 USB/Serial converter for the serial console.

    2. I have only tested this in IRAM with UART reconfiguration disabled.
       See "Sample Debug Steps".  In that case, NuttX is started via GDB.
       It has, however, been reported to me that this configuration also
       runs when written to address 0x1000 of FLASH with the esptool.py
       (as described above).  Then NuttX is started via the second level
       bootloader.  I cannot vouch for that since I have never tried it.

    3. There are open clocking issues.  Currently clock configuration
       logic is disabled because I don't have the technical information
       to provide that logic -- hopefully that is coming.  As a
       consequence, whatever clock setup was left when NuttX started is
       used.  For the case of execution out of IRAM with GDB, the
       settings in boards/xtensa/esp32/esp32-core/include/board.h work.
       To check the timing, I use a stop watch and:

         nsh> sleep 60

       If the timing is correct in the board.h header file, the value
       timed with the stop watch should be about 60 seconds.  If not,
       change the frequency in the board.h header file.

  smp:

    Another NSH configuration, similar to nsh, but also enables
    SMP operation.  It differs from the nsh configuration only in these
    additional settings:

    SMP is enabled:

      CONFIG_SMP=y
      CONFIG_SMP_IDLETHREAD_STACKSIZE=3072
      CONFIG_SMP_NCPUS=2
      CONFIG_SPINLOCK=y

    The apps/testing/smp test is included:

      CONFIG_TESTING_SMP=y
      CONFIG_TESTING_SMP_NBARRIER_THREADS=8
      CONFIG_TESTING_SMP_PRIORITY=100
      CONFIG_TESTING_SMP_STACKSIZE=2048

    NOTES:
    1. See NOTES for the nsh configuration.

  ostest:

    This is the NuttX test at apps/examples/ostest that is run against all new
    architecture ports to assure a correct implementation of the OS.  The default
    version is for a single CPU but can be modified for an SMP test by adding:

      CONFIG_SMP=y
      CONFIG_SMP_IDLETHREAD_STACKSIZE=2048
      CONFIG_SMP_NCPUS=2
      CONFIG_SPINLOCK=y

    NOTES:
    1. See NOTES for the nsh configuration.
    2. 2016-12-23: Test appears to be fully functional in the single CPU mode.
    3. 2016-12-24: But when SMP is enabled, there is a consistent, repeatable
       crash in the waitpid() test.  At the time of the crash, there is
       extensive memory corruption and a user exception occurs (cause=28).

  mmcsdspi:

    This config tests the SPI driver by connecting an SD Card reader over SPI.
    SPI2 is used and kept with the default IOMUX pins, i.e.:
        CS   --> 15
        SCK  --> 14
        MOSI --> 13
        MISO --> 12
    Once booted the following command is used to mount a FAT file system:
        mount -t vfat /dev/mmcsd0 /mnt

  spiflash:

    This config tests the external SPI that comes with an ESP32 module connected
    through SPI1.

    By default a SmartFS file system is selected.
    Once booted you can use the following commands to mount the file system:
        mksmartfs /dev/smart0
        mount -t smartfs /dev/smart0 /mnt

    Note that mksmartfs is only needed the first time.

  psram:

    This config tests the PSRAM driver over SPIRAM interface.
    You can use the ramtest command to test the PSRAM memory. We are testing
    only 64KB on this example (64 * 1024), but you can change this number to
    2MB or 4MB depending on PSRAM chip used on your board:

        nsh> ramtest -w 0x3F800000 65536
        RAMTest: Marching ones: 3f800000 65536 
        RAMTest: Marching zeroes: 3f800000 65536 
        RAMTest: Pattern test: 3f800000 65536 55555555 aaaaaaaa
        RAMTest: Pattern test: 3f800000 65536 66666666 99999999 
        RAMTest: Pattern test: 3f800000 65536 33333333 cccccccc 
        RAMTest: Address-in-address test: 3f800000 65536

Things to Do
============

  1. There is no support for an interrupt stack yet.

  2. There is no clock initialization logic in place.  This depends on logic in
     Espressif libraries.  The board comes up using that basic 40 MHz crystal
     for clocking.  Getting to 80 MHz will require clocking initialization in
     esp32_clockconfig.c.

  3. I did not implement the lazy co-processor save logic supported by Xtensa.
     That logic works like this:

     a. CPENABLE is set to zero on each context switch, disabling all co-
        processors.
     b. If/when the task attempts to use the disabled co-processor, an
        exception  occurs
     c. The co-processor exception handler re-enables the co-processor.

     Instead, the NuttX logic saves and restores CPENABLE on each context
     switch.  This has disadvantages in that (1) co-processor context will
     be saved and restored even if the co-processor was never used, and (2)
     tasks must explicitly enable and disable co-processors.

  4. Currently the Xtensa port copies register state save information from
     the stack into the TCB.  A more efficient alternative would be to just
     save a pointer to a register state save area in the TCB.  This would
     add some complexity to signal handling and also also the
     up_initialstate().  But the performance improvement might be worth
     the effort.

  5. See SMP-related issues above

  6. See OpenOCD for the ESP32 above

  7. Currently will not boot unless serial port initialization is disabled.
     This will use the serial port settings as left by the preceding
     bootloader:

     I also made this change configuration which will eliminate all attempts to
     re-configure serial. It will just use the serial settings as they were left
     by the bootloader:

       CONFIG_SUPPRESS_UART_CONFIG=y

     I have not debugged this in detail, but this appears to be an issue with the
     implementation of esp32_configgpio() and/or gpio_matrix_out() when called from
     the setup logic in arch/xtensa/src/esp32/esp32_serial.c.  I am not inclined
     to invest a lot in driver debug until the clock configuration is finalized.

     UPDATE:  This may have been fixed with PR 457:

     https://bitbucket.org/nuttx/nuttx/pull-requests/457/
       fix-esp32-gpio-enable-reg-and-default-uart/diff

     That has not yet been verified.
