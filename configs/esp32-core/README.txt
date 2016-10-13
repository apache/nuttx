README for the Expressif ESP32 Core board (V2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

  Features:

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

  The steps to build toolchain with crosstool-NG on Linux are as follows:

    git clone -b xtensa-1.22.x https://github.com/espressif/crosstool-NG.git
    cd crosstool-NG
    ./bootstrap && ./configure --prefix=$PWD && make install
    ./ct-ng xtensa-esp32-elf
    ./ct-ng build
    chmod -R u+w builds/xtensa-esp32-elf

  These steps are given in setup guide in ESP-IDF repository:
  https://github.com/espressif/esp-idf/blob/master/docs/linux-setup.rst#alternative-step-1-compile-the-toolchain-from-source-using-crosstool-ng

  NOTE: the xtensa-esp32-elf configuration is only available in the
  xtensa-1.22.x branch.
