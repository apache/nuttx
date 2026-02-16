===================================
Keeping constants in program memory
===================================

By default, all constants in a program running on AVR device are copied
into RAM. This document describes the reasons for doing that and options
to keep them in program memory.

Introduction
============

AVR architecture is a Harvard architecture, program and data memories are
accessed over separate buses with their own address spaces. While this approach
has its advantages, it does not match well with C programs which expect
to be able to access everything in the same way. (That is - using
the same instructions.) In fact, unless some measures are taken, C program
is completely unable to access initialized variables.

Consider this variable declaration and function call:

::

 const char hello[] = "Hello, world\n";
 printf(hello);

When this code is compiled, the string is stored somewhere within
the program and uploaded to the program memory (flash.) Pointer
variable ``hello`` then contains address of the string. Call
to ``printf`` receives this address and attempts to load and process
characters of this string - on AVR, indirect load from memory
instructions would be used for that.

On a von Neumann architecture, which has single address space
for both program and data, this would pose no problem. Since
the program memory is part of the overall address space, the load
instructions can reach the data contained within it. This is,
however, not the case for AVR. Anything stored
in program memory is inaccessible for regular load instruction
and the ``printf`` call would fail.

Solution of the problem
=======================

AVR provides instructions that are able to load data from program memory.
These instructions can be used to copy all the constants into the RAM,
the copy is then available to regular load instructions and the program
can work correctly.

Internally, variable ``hello`` is altered to contain address of the copy
of the string with the data address space . It can be passed freely into
``printf`` or any other function.

All that is needed is a code that performs that copy and that code
is present in NuttX. It is executed automatically at program startup.
If the application is being developed for a supported board, everything
happens automatically. (If the application is being developed for a custom board, the board's
linker script only needs to provide some variables - common architecture
code then takes care of the rest.)

In other words - by default, any application running on NuttX is able
to freely pass any variable to any NuttX interface.

Problem of the solution
=======================

As described, this solution works reliably and correctly. However, there
is a significant cost to it - it consumes RAM, which is a limited resource.
For example, one of the supported chips is ATmega128 featuring 4kB of SRAM.
Even a simple program will quickly consume significant part of it,
especially if it contains a lot of strings.

Constants in program memory
===========================

As its name suggests, this document describes techniques used
to make the program work without the need to copy constants to RAM.

Using PROGMEM
-------------

PROGMEM is a macro defined GNU's C library for AVR architecture.
It translates to a qualifier that instructs the compiler not to move
the variable from program memory to RAM. It is used like this:

::

 const uint8_t values[4] PROGMEM = { 0, 2, 4, 6 };

When the variable is declared this way, it will not be copied to RAM
by the initialization code.
It cannot be used by normal means in this state though. As a pointer,
it holds address of data in program memory. This makes anything like
the following impossible:

::

 function(values);

The issue here is described at the beginning of the document. Contents
of ``values`` is not present in the RAM and therefore cannot be reached
using regular load instructions.

Instead, the compiler needs to be explicitly instructed to read from
program memory. The C library provides functions to achieve that. Those
functions will accept the pointer to program memory and are at least
partially written in AVR assembly, making sure that LPM (load from
program memory) instructions are used.

Main drawback of this method manifests itself when a function needs
to be able to accept a parameter which may be stored either in RAM
or in the program memory. It either needs to have an additional parameter
to distinguish where to read from, or the function needs to be provided
in two variants. (Or even more variants if it accepts more than one
such parameter.)

This is unsupported in NuttX altogether. The application may use variables
declared with PROGMEM qualifier freely but must not pass them to any NuttX
interface.

Using __memx/__flashx
=====================

An example usage of the ``__memx`` qualifier would be:

::

 const __memx char hello[] = "Hello, world\n";

In this context, the qualifier has the same same meaning as PROGMEM above,
the variable will not be copied into the RAM by the initialization code.
(``__flashx`` has the same meaning but this qualifier is a feature of relatively
new - relatively of when this document is written - release of GCC
and is not used in NuttX.)

The meaning changes when the variable is used, for example:

::

 void function(const __memx char *arg)

In this case, ``__memx`` signals the compiler that the pointer may
be dereferenced to either the data or the program memory address space.
That needs to be determined during run-time.

Internally, this is achieved by extending the pointer to 24-bit length;
this also accommodates devices with more than 64kB program memory.
Most significant bit in the pointer then determines which address space
needs to be used when dereferencing the pointer. This bit is set
for data address space and cleared for program memory address space.

There is a significant run-time cost of using this method. Essentially,
every memory access to a variable with this qualifier is replaced with
a function call that determines memory type and reads from appropriate
address space. This call can take around 15 clock cycles for single
byte read. It is also entirely possible that unoptimized call
to eg. strlen will call this function for each byte in the string.

NuttX supports these qualifiers using IOBJ and IPTR macros like this:

::

 const IOBJ char hello[] = "Hello, world\n";
 void function(const IPTR char *arg);

``IOBJ`` denotes variable that should remain in program memory.
It currently translates to ``__memx`` but may eventually be switched
to ``__flashx``. ``IPTR`` always translates to ``__memx``.

This method of keeping constants in program memory
has a very limited support in NuttX. Essentially, it was
added as a debugging feature to support format strings with debug
messages. What this means is that functions related to logging
tend to have the ``IPTR`` qualifier in their declaration. Other functions
don't - most interactions with the kernel will not accept these pointers.

Note that both ``IOBJ`` and ``IPTR`` need to be activated by
:menuselection:`System Type --> Mark const variables with __memx`.
If this configuration option is not set, both macros are defined
to be empty and all strings will be copied to RAM (performance penalty
discussed above is therefore removed as well.)

Using memory-mapped flash
=========================

Newer AVR devices - tinyAVR and AVR DA/DB family - have their program
memory mapped into upper 32kB half of data memory address space.
(If the program memory size exceeds 32kB, only a 32kB-sized window
is mapped. This is controlled by NVM peripheral within the chip.
On current chips, the top window is mapped by default.)

This can be leveraged in a way that makes these AVR devices behave
as a von Neumann architecture. With proper configuration in a linker
script, all constants can be placed into the mapped program memory
region where they will be accessible for both load from program memory
instructions and load from data address space instructions.

As long as these constants fit into the 32kB window, this is a best
available option on devices that support it. It combines advantages
of all previous options and doesn't have any of their drawbacks.
The performance penalty is negligible (flash read is few cycles slower
than RAM read), RAM is not consumed and all variables are fully
available to be used as parameters for any kernel interface.

Unlike previous options, using this one is fully controlled by board's
linker script. The linker script needs to place the constants
(eg. ``rodata`` section) to appropriate memory location.

Despite that, there is still a configuration option
:menuselection:`System Type --> Use memory-mapped access to flash`,
which is selected by default on devices that support this method
of not copying data from program memory to RAM. Setting it unlocks
additional configuration options
:menuselection:`Size of .rodata FLMAP section` and
:menuselection:`Offset of .rodata FLMAP section` which may be used
to further configure section sizes. Note that these values are
only made available to the linker and board's linker script needs
to be designed to obey them.

To have these configuration options available, the board needs
to select ``AVR_HAVE_BOARD_FLMAP`` in its configuration. It declares
that its linker script will obey ``__RODATA_SIZE__`` and
``__RODATA_OFFSET__`` symbols (which are set by the above-mentioned
configuration options.)

See the linker script of :ref:`breadxavr_board` for an example.
