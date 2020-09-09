Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^
Table of Contents
^^^^^^^^^^^^^^^^^

  o Architecture-Specific Code
  o Summary of Files
  o Supported Architectures
  o Configuring NuttX

Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^

The NuttX configuration consists of:

o Processor architecture specific files.  These are the files contained
  in the arch/<arch-name>/ directory discussed in this README.

o Chip/SoC specific files.  Each processor architecture is embedded in
  chip or System-on-a-Chip (SoC) architecture.  The full chip
  architecture includes the processor architecture plus chip-specific
  interrupt logic, general purpose I/O (GPIO) logic, and specialized,
  internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the arch/<arch-name>/ directory and are selected
  via the CONFIG_ARCH_name selection

o Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as peripheral
  LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  boards/<arch>/<chip>/<board> sub-directories.

This README will address the processor architecture specific files that
are contained in the arch/<arch-name>/ directory.  The file
include/nuttx/arch.h identifies all of the APIs that must be provided by
this architecture specific logic.  (It also includes
arch/<arch-name>/arch.h as described below).

Directory Structure
^^^^^^^^^^^^^^^^^^^

The arch/ directory contains architecture-specific logic.  The complete
board port is defined by the architecture-specific code in this
directory plus the board-specific configurations in the boards/
directory.  Each architecture must provide a subdirectory <arch-name>
under arch/ with the following characteristics:


        <arch-name>/
        |-- include/
        |   |--<chip-name>/
        |   |  `-- (chip-specific header files)
        |   |--<other-chips>/
        |   |-- arch.h
        |   |-- irq.h
        |   `-- types.h
        `-- src/
            |--<chip-name>/
            |  `-- (chip-specific source files)
            |--<other-chips>/
            |-- Makefile
            `-- (architecture-specific source files)

Summary of Files
^^^^^^^^^^^^^^^^

include/<chip-name>/
  This sub-directory contains chip-specific header files.

include/arch.h
  This is a hook for any architecture specific definitions that may be
  needed by the system.  It is included by include/nuttx/arch.h

include/types.h
  This provides architecture/toolchain-specific definitions for standard
  types.  This file should typedef:

    _int8_t, _uint8_t, _int16_t, _uint16_t, _int32_t, _uint32_t

  and if the architecture supports 64-bit integers:

    _int24_t, _uint24_t, _int64_t, _uint64_t

  NOTE that these type names have a leading underscore character.  This
  file will be included (indirectly) by include/stdint.h and typedef'ed
  to the final name without the underscore character.  This roundabout
  way of doings things allows the stdint.h to be removed from the
  include/ directory in the event that the user prefers to use the
  definitions provided by their toolchain header files.

    irqstate_t

  Must be defined to the size required to hold the interrupt
  enable/disable state.

  This file will be included by include/sys/types.h and be made
  available to all files.

include/irq.h
  This file needs to define some architecture-specific functions
  (usually inline if the compiler supports inlining) and structures.
  These include:

  - struct xcptcontext.  This structure represents the saved context of
    a thread.

  - irqstate_t up_irq_save(void) -- Used to disable all interrupts.

  - void up_irq_restore(irqstate_t flags) -- Used to restore interrupt
    enables to the same state as before up_irq_save was called.

  NOTE: These interfaces are not available to application code but can
  only be used within the operating system code.  And, in general, these
  functions should *never* be called directly, not unless you know
  absolutely well what you are doing.  Rather you should typically use
  the wrapper functions enter_critical_section() and
  leave_critical_section() as prototyped in include/nuttx/irq.h.

  This file must also define NR_IRQS, the total number of IRQs supported
  by the board.

src/<chip-name>/
  This sub-directory contains chip-specific source files.

src/Makefile
  This makefile will be executed to build the targets src/libup.a and
  src/up_head.o.  The up_head.o file holds the entry point into the
  system (power-on reset entry point, for example).  It will be used in
  the final link with libup.a and other system archives to generate the
  final executable.

Supported Architectures
^^^^^^^^^^^^^^^^^^^^^^^
NOTE: nuttx/Documentation/NuttX.html for current information about the
state of these MCU ports.

arch/sim - Linux/Cygwin simulation
    A user-mode port of NuttX to the x86 Linux platform is available.
    The purpose of this port is primarily to support OS feature
    development.  This port does not support interrupts or a real timer
    (and hence no round robin scheduler).  Otherwise, it is complete.

arch/arm - ARM-based micro-controllers
    This directory holds common ARM architectures.  At present, this
    includes the following subdirectories:

    Architecture Support
      arch/arm/include and arch/arm/src/common
      arch/arm/src/arm and arch/arm/include/arm
      arch/arm/src/armv7-a and arch/arm/include/armv6-m
      arch/arm/src/armv7-a and arch/arm/include/armv7-a
      arch/arm/src/armv7-m and arch/arm/include/armv7-m
      arch/arm/src/armv7-r and arch/arm/include/armv7-r

    MCU support
      arch/arm/include/a1x and arch/arm/src/a1x
      arch/arm/include/am335x and arch/arm/src/am335x
      arch/arm/include/c5471 and arch/arm/src/c5471
      arch/arm/include/cxd56xx and arch/arm/src/cxd56xx
      arch/arm/include/dm320 and arch/arm/src/dm320
      arch/arm/include/efm32 and arch/arm/src/efm32
      arch/arm/include/imx1 and arch/arm/src/imx1
      arch/arm/include/imx6 and arch/arm/src/imx6
      arch/arm/include/imxrt and arch/arm/src/imxrt
      arch/arm/include/kinetis and arch/arm/src/kinetis
      arch/arm/include/kl and arch/arm/src/kl
      arch/arm/include/lc823450 and arch/arm/src/lc823450
      arch/arm/include/lpc17xx_40xx and arch/arm/src/lpc17xx_40xx
      arch/arm/include/lpc214x and arch/arm/src/lpc214x
      arch/arm/include/lpc2378 and arch/arm/src/lpc2378
      arch/arm/include/lpc31xx and arch/arm/src/lpc31xx
      arch/arm/include/lpc43xx and arch/arm/src/lpc43xx
      arch/arm/include/lpc54xx and arch/arm/src/lpc54xx
      arch/arm/include/max326xx and arch/arm/src/max326xx
      arch/arm/include/moxart and arch/arm/src/moxart
      arch/arm/include/nrf52 and arch/arm/src/nrf52
      arch/arm/include/nuc1xx and arch/arm/src/nuc1xx
      arch/arm/include/s32k1xx and arch/arm/src/s32k1xx
      arch/arm/include/sam34 and arch/arm/src/sam34
      arch/arm/include/sama5 and arch/arm/src/sama5
      arch/arm/include/samd2l2 and arch/arm/src/samd2l2
      arch/arm/include/samd5e5 and arch/arm/src/samd5e5
      arch/arm/include/samv7 and arch/arm/src/samv7
      arch/arm/include/stm32 and arch/arm/src/stm32
      arch/arm/include/stm32f0l0g0 and arch/arm/src/stm32f0l0g0
      arch/arm/include/stm32f7 and arch/arm/src/stm32f7
      arch/arm/include/stm32h7 and arch/arm/src/stm32h7
      arch/arm/include/stm32l4 and arch/arm/src/stm32l4
      arch/arm/include/str71x and arch/arm/src/str71x
      arch/arm/include/tiva and arch/arm/src/tiva
      arch/arm/include/tms570 and arch/arm/src/tms570
      arch/arm/include/xmc4 and arch/arm/src/xmc4

arch/avr
    This directory is dedicated to ports to the Atmel AVR (8-bit) and
    AVR32 (32-bit) MCU families.  STATUS:  Under development.

    Architecture Support
      arch/avr/include/avr and arch/avr/src/avr
      arch/avr/include/avr32 and arch/avr/src/avr32

    MCU support
      arch/avr/include/atmega and arch/avr/src/atmega
      arch/avr/include/at90usb and arch/avr/src/at90usb
      arch/avr/include/at32uc3 and arch/avr/src/at32uc3
      arch/avr/include/xmega and arch/avr/src/xmega

arch/hc
    This directory is dedicated to ports to the Freescale HC family.

    arch/arm/include/m9s12 and arch/arm/src/m9s12

arch/mips
    This directory is dedicated to ports to the MIPS family.

    Architecture Support
      arch/mips/include/mips32 and arch/mips/src/mips32

    MCU support
      arch/mips/include/pic32mx and arch/mips/src/pic32mx
      arch/mips/include/pic32mz and arch/mips/src/pic32mz

arch/misoc
    This directory is dedicated to ports to the Misoc family.

    MCU support
      arch/misoc/include/lm32 and arch/misoc/src/lm32
      arch/misoc/include/minerva and arch/misoc/src/minerva

arch/renesas - Support for Renesas and legacy Hitachi microcontrollers.
    This include SuperH and M16C.

    Architecture Support
      arch/renesas/include and arch/renesas/src/common

    MCU support
      arch/renesas/include/m16c and arch/renesas/src/m16c
      arch/renesas/include/rx65n and arch/renesas/src/rx65n
      arch/renesas/include/sh1 and arch/renesas/src/sh1

arch/or1k
    This directory is dedicated to ports to OpenRISC architectures.

    arch/or1k/include/mor1k and arch/or1k/src/mor1k

arch/risc-v
    This directory is dedicated to ports to the RISC-V family.

    Architecture Support
      arch/risc-v/include/rv32im

    MCU support
      arch/risc-v/include/fe310 and arch/risc-v/src/fe310
      arch/risc-v/include/gap8 and arch/risc-v/src/gap8
      arch/risc-v/include/k210 and arch/risc-v/src/k210
      arch/risc-v/include/litex and arch/risc-v/src/litex
      arch/risc-v/include/nr5m100 and arch/risc-v/src/nr5m100
      arch/risc-v/include/rv32im and arch/risc-v/src/rv32im
      arch/risc-v/include/rv64gc and arch/risc-v/src/rv64gc

arch/x86 - Intel x86 architectures
    This directory holds related, 32- and 64-bit architectures from
    Intel.  At present, this includes the following subdirectories:

    Architecture Support
      arch/x86/include and arch/x86/src/common

    MCU support
      arch/x86/include/i486 and arch/x86/src/i486
      arch/x86/include/qemu and arch/x86/src/qemu

arch/x86_64 - Intel x86 64-bit architectures
    This directory holds related 64-bit architectures from Intel.  At
    present, this includes the following subdirectories:

    Architecture Support
      arch/x86_64/include and arch/x86_64/src/common

    MCU support
      arch/x86_64/include/intel64 and arch/x86_64/src/intel64
      arch/x86_64/include/qemu and arch/x86_64/src/qemu

arch/xtensa
    Implementations based on the Cadence® Tensilica® Xtensa® processors,
    such as the Xtensa LX6 dataplane processing units (DPUs).  At
    present, this includes the following subdirectories:

    Common XTENSA support:
      arch/xtensa/include and arch/xtensa/src/common

    LX6 DPU support:
      arch/xtensa/include/lx6 and arch/xtensa/xtensa/lx6

    Espressif ESP32 implementation of the LX6 DPU:
      arch/xtensa/include/esp32 and arch/xtensa/xtensa/esp32

arch/z16 - ZiLOG 16-bit processors
    This directory holds related, 16-bit architectures from ZiLOG.  At
    present, this includes the following subdirectories:

    Architecture Support
      arch/z16/include and arch/z16/src/common

    MCU support
      arch/z16/include/z16f and arch/z16/src/z16f

arch/z80 - ZiLOG 8-bit microcontrollers
    This directory holds related, 8-bit architectures from ZiLOG.  At
    present, this includes the following subdirectories:

    Architecture Support
      arch/z80/include and arch/z80/src/common

    MCU support
      arch/z80/include/ez80 and arch/z80/src/ez80
      arch/z80/include/z80 and arch/z80/src/z180
      arch/z80/include/z8 and arch/z80/src/z8
      arch/z80/include/z80 and arch/z80/src/z80
