lib
===

This directory contains numerous, small functions typically associated with
what you would expect to find in a standard C library.  The sub-directories
in this directory contain standard interface that can be executed by user-
mode programs.

Normally, NuttX is built with no protection and all threads running in kerne-
mode.  In that model, there is no real architectural distinction between
what is a kernel-mode program and what is a user-mode program; the system is
more like on multi-threaded program that all runs in kernel-mode.

But if the CONFIG_BUILD_PROTECTED option is selected, NuttX will be built
into distinct user-mode and kernel-mode sections.  In that case, most of the
code in the nuttx/ directory will run in kernel-mode with exceptions
of (1) the user-mode "proxies" found in syscall/proxies, and (2) the
standard C library functions found in this directory.  In this build model,
it is critical to separate the user-mode OS interfaces in this way.

If CONFIG_BUILD_KERNEL is selected, then only a NuttX kernel will be built
with no applications.

Sub-Directories
===============

The files in the libs/libc/ directory are organized (mostly) according which file
in the include/ directory provides the prototype for library functions.  So
we have:

  audio     - This part of the audio system: nuttx/audio/audio.h
  builtin   - Support for builtin applications.  Used by nuttx/binfmt and NSH.
  dlfcn     - dlfcn.h
  endian    - endian.h
  errno     - errno.h
  hex2bin   - hex2bin.h
  libgen    - libgen.h
  locale    - locale.h
  lzf       - lzf.h
  fixedmath - fixedmath.h
  grp       - grp.h
  inttypes  - inttypes.h
  machine   - Various architecture-specific implementations.
  math      - math.h
  modlib    - Part of module and shared library logic: nuttx/lib/modlib.h
  net       - Various network-related header files: netinet/ether.h, arpa/inet.h
  pthread   - pthread.h
  pwd       - pwd.h
  queue     - queue.h
  sched     - sched.h
  semaphore - semaphore.h
  stdio     - stdio.h
  stdlib    - stdlib.h
  string    - string.h (and legacy strings.h and non-standard nuttx/b2c.h)
  time      - time.h
  uio       - sys/uio.h
  unistd    - unistd.h
  wchar     - wchar.h
  wctype    - wctype.h

Most of these are "standard" header files; some are not: hex2bin.h and
fixemath.h are non-standard.

There is also a misc/ subdirectory that contains various internal functions
and interfaces from header files that are too few to warrant their own sub-
directory:

  misc      - Nonstandard "glue" logic, debug.h, crc32.h, dirent.h

Library Database
================

Information about functions available in the NuttX C library information is
maintained in a database.  That "database" is implemented as a simple comma-
separated-value file, libc.csv.  Most spreadsheets programs will accept this
format and can be used to maintain the library database.

This library database will (eventually) be used to generate symbol library
symbol table information that can be exported to external applications.

The format of the CSV file for each line is:

  Field 1: Function name
  Field 2: The header file that contains the function prototype
  Field 3: Condition for compilation
  Field 4: The type of function return value.
  Field 5 - N+5: The type of each of the N formal parameters of the function

Each type field has a format as follows:

  type name:
        For all simpler types
  formal type | actual type:
        For array types where the form of the formal (eg. int parm[2])
        differs from the type of actual passed parameter (eg. int*).  This
        is necessary because you cannot do simple casts to array types.
  formal type | union member actual type | union member fieldname:
        A similar situation exists for unions.  For example, the formal
        parameter type union sigval -- You cannot cast a uintptr_t to
        a union sigval, but you can cast to the type of one of the union
        member types when passing the actual parameter.  Similarly, we
        cannot cast a union sigval to a uinptr_t either.  Rather, we need
        to cast a specific union member fieldname to uintptr_t.

NOTE: The tool mksymtab can be used to generate a symbol table from this CSV
file.  See nuttx/tools/README.txt for further details about the use of mksymtab.

symtab
======

Symbol Tables and Build Modes
-----------------------------
This directory provide support for a symbol table which provides all/most of
system and C library services/functions to the application and NSH.

Symbol tables have differing usefulness in different NuttX build modes:

  1. In the FLAT build (CONFIG_BUILD_FLAT), symbol tables are used to bind
     addresses in loaded ELF or NxFLAT modules to base code that usually
     resides in FLASH memory.  Both OS interfaces and user/application
     libraries are made available to the loaded module via symbol tables.

  2. Symbol tables may be of value in a protected build
     (CONFIG_BUILD_PROTECTED) where the newly started user task must
     share resources with other user code (but should use system calls to
     interact with the OS).

  3. But in the kernel build mode (CONFIG_BUILD_LOADABLE), only fully linked
     executables loadable via execl(), execv(), or posix_spawan() can used.
     There is no use for a symbol table with the kernel build since all
     memory resources are separate; nothing is share-able with the newly
     started process.

Code/Text Size Implications
---------------------------
The option can have substantial effect on system image size, mainly
code/text.  That is because the instructions to generate symtab.inc
above will cause EVERY interface in the NuttX RTOS and the C library to be
included into build.  Add to that the size of a huge symbol table.

In order to reduce the code/text size, you may want to manually prune the
auto-generated symtab.inc file to remove all interfaces that you do
not wish to include into the base FLASH image.
