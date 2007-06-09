Board-Specific Configurations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Table of Contents
^^^^^^^^^^^^^^^^^

  o Board-Specific Configurations
  o Summary of Files
  o Supported Architectures
  o Configuring NuttX

Board-Specific Configurations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The NuttX configuration consists of:

o Processor architecture specific files.  These are the files contained
  in the arch/<arch-name>/ directory.

o Chip/SoC specific files.  Each processor processor architecture
  is embedded in chip or System-on-a-Chip (SoC) architecture.  The
  full chip architecture includes the processor architecture plus
  chip-specific interrupt logic, general purpose I/O (GIO) logic, and
  specialized, internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the arch/<arch-name>/ directory and are selected
  via the CONFIG_ARCH_name selection

o Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as
  peripheral LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  configs/<board-name>/ sub-directories and are discussed in this
  README.

The configs/ subdirectory contains configuration data for each board.  These
board-specific configurations plus the architecture-specific configurations in
the arch/ subdirectory completely define a customized port of NuttX.

Directory Structure
^^^^^^^^^^^^^^^^^^^

The configs directory contains board specific configurationlogic.  Each
board must provide a subdirectory <board-name> under configs/ with the
following characteristics:


	<board-name>
	|-- include/
	|   `-- (board-specific header files)
	|-- src/
	|   |-- Makefile
	|   `-- (board-specific source files)
	|-- Make.defs
	|-- defconfig
	`-- setenv.sh

Summary of Files
^^^^^^^^^^^^^^^^

include/ -- This directory contains board specific header files.  This
  directory will be linked as include/arch/board at configuration time and
  can be included via '#include <arch/board/header.h>'.  These header file
  can only be included by files in arch/<arch-name>include/ and
  arch/<arch-name>/src

src/ -- This directory contains board specific drivers.  This
  directory will be linked as arch/<arch-name>/src/board at configuration
  time and will be integrated into the build system.

src/Makefile -- This makefile will be invoked to build the board specific
  drivers.  It must support the following targets:  libext$(LIBEXT), clean,
  and distclean.

Make.defs -- This makefile fragment provides architecture and
  tool-specific build options.  It will be included by all other
  makefiles in the build (once it is installed).  This make fragment
  should define:

	Tools: CC, LD, AR, NM, OBJCOPY, OBJDUMP
	Tool options: CFLAGS, LDFLAGS

  When this makefile fragment runs, it will be passed TOPDIR which
  is the path to the root directory of the build.  This makefile
  fragment may include ${TOPDIR}/.config to perform configuration
  specific settings.  For example, the CFLAGS will most likely be
  different if CONFIG_DEBUG=y.

defconfig -- This is a configuration file similar to the Linux
  configuration file.  In contains varialble/value pairs like:

	CONFIG_VARIABLE=value

  This configuration file will be used at build time:

    (1) as a makefile fragment included in other makefiles, and
    (2) to generate include/nuttx/config.h which is included by
        most C files in the system.

  The following variables are recognized by the build (you may
  also include architecture/board-specific settings).

	Architecture selection:

		CONFIG_ARCH - Identifies the arch/ subdirectory
		CONFIG_ARCH_name - For use in C code
		CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory
		CONFIG_ARCH_CHIP_name - For use in C code
		CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
		   hence, the board that supports the particular chip or SoC.
		CONFIG_ARCH_BOARD_name - For use in C code
                CONFIG_ENDIAN_BIG - define if big endian (default is little
                   endian)

	Some architectures require a description of the the RAM configuration:

		CONFIG_DRAM_SIZE - Describes the installed DRAM.
		CONFIG_DRAM_START - The start address of DRAM (physical)
		CONFIG_DRAM_VSTART - The startaddress of DRAM (virtual)

	General OS setup

		CONFIG_EXAMPLE - identifies the subdirectory in examples
		  that will be used in the build
		CONFIG_DEBUG - enables built-in debug options
		CONFIG_DEBUG_VERBOSE - enables verbose debug output
		CONFIG_HAVE_LOWPUTC - architecture supports low-level, boot
		  time console output
		CONFIG_MM_REGIONS - If the architecture includes multiple
		  regions of memory to allocate from, this specifies the
		  number of memory regions that the memory manager must
		  handle and enables the API mm_addregion(start, end);
		CONFIG_TICKS_PER_MSEC - The default system timer is 100Hz
		  or TICKS_PER_MSEC=10.  This setting may be defined to
		  inform NuttX that the processor hardware is providing
		  system timer interrupts at some interrupt interval other
		  than 10 msec.
		CONFIG_RR_INTERVAL - The round robin timeslice will be set
		  this number of milliseconds;  Round robin scheduling can
		  be disabled by setting this value to zero.
		CONFIG_SCHED_INSTRUMENTATION - enables instrumentation in 
		  scheduler to monitor system performance
		CONFIG_TASK_NAME_SIZE - Spcifies that maximum size of a
		  task name to save in the TCB.  Useful if scheduler
		  instrumentation is selected.  Set to zero to disable.
		CONFIG_START_YEAR, CONFIG_START_MONTH, CONFIG_START_DAY -
		  Used to initialize the internal time logic.
		CONFIG_JULIAN_TIME - Enables Julian time conversions
		CONFIG_DEV_CONSOLE - Set if architecture-specific logic
		  provides /dev/console.  Enables stdout, stderr, stdin.

	The following can be used to disable categories of APIs supported
	by the OS.  If the compiler supports weak functions, then it
	should not be necessary to disable functions unless you want to
	restrict usage of those APIs.

	There are certain dependency relationships in these features.

	o mq_notify logic depends on signals to awaken tasks
	  waiting for queues to become full or empty.
	o pthread_condtimedwait() depends on signals to wake
	  up waiting tasks.

		CONFIG_DISABLE_CLOCK, CONFIG_DISABLE_POSIX_TIMERS, CONFIG_DISABLE_PTHREAD.
		CONFIG_DISABLE_SIGNALS, CONFIG_DISABLE_MQUEUE, CONFIG_DISABLE_MOUNTPOUNT


	Misc libc settings

		CONFIG_NOPRINTF_FIELDWIDTH - sprintf-related logic is a
		   little smaller if we do not support fieldwidthes

	Allow for architecture optimized implementations

		The architecture can provide optimized versions of the
		following to improve sysem performance

		CONFIG_ARCH_MEMCPY, CONFIG_ARCH_MEMCMP, CONFIG_ARCH_MEMMOVE
		CONFIG_ARCH_MEMSET, CONFIG_ARCH_STRCMP, CONFIG_ARCH_STRCPY
		CONFIG_ARCH_STRNCPY, CONFIG_ARCH_STRLEN, CONFIG_ARCH_BZERO
		CONFIG_ARCH_KMALLOC, CONFIG_ARCH_KZMALLOC, CONFIG_ARCH_KFREE

	Sizes of configurable things (0 disables)

		CONFIG_MAX_TASKS - The maximum number of simultaneously
		  active tasks.  This value must be a power of two.
		CONFIG_NPTHREAD_KEYS - The number of items of thread-
		  specific data that can be retained
		CONFIG_NFILE_DESCRIPTORS - The maximum number of file
		  descriptors (one for each open)
		CONFIG_NFILE_STREAMS - The maximum number of streams that
		  can be fopen'ed
		CONFIG_NAME_MAX - The maximum size of a file name.
		CONFIG_STDIO_BUFFER_SIZE - Size of the buffer to allocate
		  on fopen. (Only if CONFIG_NFILE_STREAMS > 0)
		CONFIG_NUNGET_CHARS - Number of characters that can be
		  buffered by ungetc() (Only if CONFIG_NFILE_STREAMS > 0)
		CONFIG_PREALLOC_MQ_MSGS - The number of pre-allocated message
		  structures.  The system manages a pool of preallocated
		  message structures to minimize dynamic allocations
		CONFIG_MQ_MAXMSGSIZE - Message structures are allocated with
		  a fixed payload size given by this settin (does not include
		  other message structure overhead.
		CONFIG_PREALLOC_WDOGS - The number of pre-allocated watchdog
		  structures.  The system manages a pool of preallocated
		  watchdog structures to minimize dynamic allocations

	Stack and heap information

		CONFIG_BOOT_FROM_FLASH - Some configurations support XIP
		  operation from FLASH.
		CONFIG_STACK_POINTER - The initial stack pointer
		CONFIG_PROC_STACK_SIZE - The size of the initial stack
		CONFIG_PTHREAD_STACK_MIN - Minimum pthread stack size
		CONFIG_PTHREAD_STACK_DEFAULT - Default pthread stack size
		CONFIG_HEAP_BASE - The beginning of the heap
		CONFIG_HEAP_SIZE - The size of the heap

setenv.sh -- This is a script that you can include that will be installed at
  the toplevel of the directory structure and can be sourced to set any
  necessary environment variables.

Supported Boards
^^^^^^^^^^^^^^^^

configs/sim
    A user-mode port of NuttX to the x86 Linux platform is available.
    The purpose of this port is primarily to support OS feature developement.
    This port does not support interrupts or a real timer (and hence no
    round robin scheduler)  Otherwise, it is complete.

configs/c5471evm
    This is a port to the Spectrum Digital C5471 evaluation board.  The
    C5471 is a dual core processor from TI with an ARM7TDMI general purpose
    processor and a c54 SDP.  NuttX runs on the ARM core and is built with
    with a GNU arm-elf toolchain*. This port is complete, verified, and
    included in the NuttX release.

configs/mcu123-lpc214x
    This is a port to the mcu123.com lpc214x development board.
    This OS is also built with the the arm-elf toolchain*

configs/ntosd-dm320
    This port uses the Neuros OSD with a GNU arm-elf toolchain*:
    see http://wiki.neurostechnology.com/index.php/Developer_Welcome .
    NuttX operates on the ARM9EJS of this dual core processor.
    STATUS: This port is code complete, verified, and included in the
    NuttX 0.2.1 release.

configs/mcu123-lpc214x
    This port is for the NXP LPC2148 as provided on the mcu123.com
    lpc214x development board.
    STATUS: This port is in progress and should be available in the
    nuttx-0.2.5 release.

configs/m68322evb
    This is a work in progress for the venerable m68322evb board from
    Motorola. This OS is also built with the the arm-elf toolchain*.

configs/pjrc-8051
    8051 Microcontroller.  This port uses the PJRC 87C52 development system
    and the SDCC toolchain.   This port is not quite ready for prime time.

Other ports for the for the TI TMS320DM270, M683222 and for MIPS are in various
states of progress

Configuring NuttX
^^^^^^^^^^^^^^^^^

Configuring NuttX requires only copying

  configs/<board-name>/Make.def to ${TOPDIR}/Make.defs
  configs/<board-name>/setenv.sh to ${TOPDIR}/setenv.sh
  configs/<board-name>/defconfig to ${TOPDIR}/.config

There is a script that automates these steps.  The following steps will
accomplish the same configuration:

  cd tools
  ./configure.sh <board-name>


