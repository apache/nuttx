symtab
======

This directory provide support for canned symbol table which provides
all/most of system and libc services/functions to the application and NSH.

The support is selected by CONFIG_LIBC_SYMTAB option and table has to be
prepared in advance manually.

It can be prepared from NuttX top level directory by next commands

  cat syscall/syscall.csv libc/libc.csv | sort >libc/symtab/canned_symtab.csv
  tools/mksymtab libc/symtab/canned_symtab.csv libc/symtab/canned_symtab.inc

Next code selectes canned symtab from application:

  #include <nuttx/binfmt/canned_symtab.h>
  ...
    canned_symtab_initialize();

The option can have substantial effect on system image size, mainly
code/text.  That is because the instructions to generate canned_symtab.inc
above will cause EVERY interface in the NuttX RTOS and the C library to be
included into build.  Add to that the size of a huge symbol table.

In order to reduce the code/text size, you may want to manually prune the
auto-generated canned_symtab.inc file to remove all interfaces that you do
not wish to include into the base FLASH image.
