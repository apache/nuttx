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

The file include/nuttx/arch.h identifies all of the APIs that must
be provided by the architecture specific logic.  (It also includes
arch/<arch-name>/arch.h as described below).

Directory Structure
^^^^^^^^^^^^^^^^^^^

Thie arch directory contains architecture specific logic.  Each aructure
provide a a subdirectory <arch-name>under arch/ with the folling characteristics:


	<arch-name>
	|-- Make.defs
	|-- defconfig
	|-- setenv.sh
	|-- include
	|   |-- arch.h
	|   |-- irq.h
	|   `-- types.h
	`-- src
	    |-- Makefile
	    `-- (architecture-specific source files)

Summary of Files
^^^^^^^^^^^^^^^^

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
  also include architecture-specific settings).

	Architecture selection:

		CONFIG_ARCH - identifies the arch subdirectory
		CONFIG_ARCH_name - for use in C code

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

		CONFIG_DISABLE_CLOCK, CONFIG_DISABLE_PTHREAD.
		CONFIG_DISABLE_SIGNALS, CONFIG_DISABLE_MQUEUE


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

include/arch.h
  This is a hook for any architecture specific definitions that may
  be needed by the system.  It is included by include/nuttx/arch.h

include/types.h
  This provides architecture/toolchain-specific definitions for
  standard types.  This file should typedef:

    sbyte, ubyte, uint8, boolean, sint16, uint16, sint32, uint32

  and

    sint64, uint64

  if the architecture supports 64-bit integers.

    irqstate_t

  Must be defined to the be the size required to hold the interrupt
  enable/disable state.

  This file will be included by include/sys/types.h and be made
  available to all files.

include/irq.h
  This file needs to define some architecture specific functions (usually
  inline if the compiler supports inlining) and structure.  These include:

  - struct xcptcontext.  This structures represents the saved context
    of a thread.

  - irqstate_t irqsave(void) -- Used to disable all interrupts.

  - void irqrestore(irqstate_t flags) -- Used to restore interrupt
    enables to the same state as before irqsave was called.

  This file must also define NR_IRQS, the total number of IRQs supported
  by the board.

src/Makefile
  This makefile will be executed to build the targets src/libup.a and
  src/up_head.o.  The up_head.o file holds the entry point into the system
  (power-on reset entry point, for example).  It will be used in
  the final link with libup.a and other system archives to generate the
  final executable.

Supported Architectures
^^^^^^^^^^^^^^^^^^^^^^^

arch/c5471
    TI TMS320C5471 (also called TMS320DM180).
    NuttX operates on the ARM7 of this dual core processor. This port
    uses the Spectrum Digital evaluation board with a GNU arm-elf toolchain*.
    This port is in progress and partially functional (However, my board
    is dead at the moment so it will be awhile before I fix it).

arch/pjrc-8051
    8051 Microcontroller.  This port uses the PJRC 87C52 development system
    and the SDCC toolchain.   This port is not quite ready for prime time.

arch/sim
  x86 Linux Simulation</b>.  Fully functional.

Other ports for the for the TI TMS320DM270 and for MIPS are in various states
of progress

Configuring NuttX
^^^^^^^^^^^^^^^^^

Configuring NuttX requires only copying

  arch/<arch-name>/Make.def to ${TOPDIR}/Make.defs
  arch/<arch-name>/setenv.sh to ${TOPDIR}/setenv.sh
  arch/<arch-name>/defconfig to ${TOPDIR}/.config

There is a script that automates these steps.  The following steps will
accomplish the same configuration:

  cd tools
  ./configure.sh <arch-name>


