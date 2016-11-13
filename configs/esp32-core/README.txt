README for the Expressif ESP32 Core board (V2)
==============================================

  The ESP32 is a dual-core system from Expressif with two Harvard
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

  o STATUS
  o ESP32 Features
  o ESP32 Toolchain
  o Memory Map
  o Serial Console
  o Buttons and LEDs
  o SMP
  o Debug Issues
  o Configurations
  o Things to Do

STATUS
======

  The basic port is underway.  No testing has yet been performed.

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

    git clone -b xtensa-1.22.x https://github.com/espressif/crosstool-NG.git
    cd crosstool-NG
    ./bootstrap && ./configure --prefix=$PWD && make install
    ./ct-ng xtensa-esp32-elf
    ./ct-ng build
    chmod -R u+w builds/xtensa-esp32-elf

  These steps are given in setup guide in ESP-IDF repository:
  https://github.com/espressif/esp-idf/blob/master/docs/linux-setup.rst#alternative-step-1-compile-the-toolchain-from-source-using-crosstool-ng

  NOTE: The xtensa-esp32-elf configuration is only available in the
  xtensa-1.22.x branch.

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
  FLASH mapped data: 0x3f400010 0x3fc00010  R   dram_0_seg
  COMMON data RAM:   0x3ffb0000 0x40000000  RW  dram_0_seg (NOTE 1,2)
  IRAM for PRO cpu:  0x40080000 0x400a0000  RX  iram0_0_seg
  RTC fast memory:   0x400c0000 0x400c2000  RWX rtc_iram_seg
  FLASH:             0x400d0018 0x40400018  RX  iram0_2_seg (actually FLASH)
  RTC slow memory:   0x50000000 0x50001000  RW  rtc_slow_seg (NOTE 3)

  NOTE 1: Linker script will reserve space at the beginning of the segment
          for BT and at the end for trace memory.
  NOTE 2: Heap enads at the top of dram0_0_seg
  NOTE 3: Linker script will reserve space at the beginning of the segment
          for co-processor reserve memory and at the end for ULP coprocessor
          reserve memory.

Serial Console
==============

  USART0 is, by default, the serial console.  It connects to the on-board
  CP2102 converter and is available on the USB connector USB CON8 (J1).

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
  and USB activity.  None of these are available for use by sofware.

SMP
===

  The ESP32 has 2 CPUs.  Support is included for testing an SMP configuration.
  That configuration is still not yet ready for usage but can be enabled with
  the following configuration settings:

    RTOS Features -> Tasks and Scheduling
      CONFIG_SPINLOCK=y
      CONFIG_SMP=y
      CONFIG_SMP_NCPUS=2
      CONFIG_SMP_IDLETHREAD_STACKSIZE=2048

  Open Issues:

  1. Currently all device interrupts are handled on the PRO CPU only.  Critical
     sections will attempt to disable interrupts but will now disable interrupts
     only on the current CPU (which may not be CPU0).  Perhaps that should be a
     spinlock to prohibit execution of interrupts on CPU0 when other CPUs are in
     a critical section?

  2. Cache Issues.  I have not though about this yet, but certainly caching is
     an issue in an SMP system:

     - Cache coherency.  Are there separate caches for each CPU?  Or a single
       shared cache?  If the are separate then keep the caches coherent will
       be an issue.
     - Caching MAY interfere with spinlocks as they are currently implemented.
       Waiting on a cached copy of the spinlock may result in a hang or a
       failure to wait.

  3. Assertions.  On a fatal assertions, other CPUs need to be stopped.

Debug Issues
============

  I basically need the debug environment and a step-by-step procedure.

    - First in need some debug environment which would be a JTAG emulator
      and software.

    - I don't see any way to connect JTAG to the ESP32 Core V2 board. There
      is a USB/Serial converter chip, but that does not look like it
      supports JTAG.

      It may be necessary to make cable.  Refer to
      http://www.esp32.com/viewtopic.php?t=381 "How to debug ESP32 with
      JTAG / OpenOCD / GDB 1st part connect the hardware."

    - I need to understand how to use the secondary bootloader.  My
      understanding is that it will configure hardware, read a partition
      table at address 0x5000, and then load code into memory.  I do need to
      download and build the bootloader?

    - Do I need to create a partition table at 0x5000?  Should this be part
      of the NuttX build?

  I see https://github.com/espressif/esp-idf/tree/master/components/bootloader
  and https://github.com/espressif/esp-idf/tree/master/components/partition_table.
  I suppose some of what I need is in there, but I am not sure what I am
  looking at right now.

  There is an OpenOCD port here: https://github.com/espressif/openocd-esp32
  and I see some additional OpenOCD documentation in
  https://github.com/espressif/esp-idf/tree/master/docs.  This documentation
  raises some more questions.    It says I need to use and external JTAG like
  the TIAO USB Multi-protocol Adapter and the Flyswatter2.  I don't have
  either of those.  I am not sure if I have any USB serial JTAG.  I have some
  older ones that might work, however.

  My understanding when I started this was that I could use my trusty Segger
  J-Link.  But that won't work with OpenOCD.  Is the J-Link that also a
  possibility?

  I also see that I can now get an ESP32 board from Sparkfun:
  https://www.sparkfun.com/products/13907 But I don't see JTAG there either:
  https://cdn.sparkfun.com/assets/learn_tutorials/5/0/7/esp32-thing-schematic.pdf

  Right now, the NuttX port depends on the bootloader to initialize hardware,
  including basic (slow) clocking.    If I had the clock configuration logic,
  would I be able to run directly out of IRAM without a bootloader?  That
  might be a simpler bring-up.

Configurations
==============

  Common Configuration Information
  --------------------------------
  Each ESP32 core configuration is maintained in sub-directories and
  can be selected as follow:

    cd tools
    ./configure.sh esp32-core/<subdir>
    cd -
    make oldconfig
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and
  perform edits as necessary so that TOOLCHAIN_BIN is the correct path to
  the directory than holds your toolchain binaries.

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
     output on [To be provided].

  Configuration sub-directories
  -----------------------------

  nsh:

    Configures the NuttShell (nsh) located at apps/examples/nsh.

    NOTES:

  smp:

    Another NSH configuration, similar to nsh, but also enables
    SMP operation.

    NOTES:

Things to Do
============

  1. There is no support for an interrupt stack yet.

  2. There is no clock intialization logic in place.  This depends on logic in
     Expressif libriaries.  The board comes up using that basic 40 Mhz crystal
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
     add some complexity to signal handling and also also the the
     up_initialstate().  But the performance improvement might be worth
     the effort.

  5. See SMP-related issues above

  6. See Debug Issues above
