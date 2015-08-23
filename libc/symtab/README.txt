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
