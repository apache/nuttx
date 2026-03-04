================================================
``mksymtab.c``, ``cvsparser.c``, ``cvsparser.h``
================================================

This is a C file that is used to build symbol tables from comma separated
value (CSV) files.  This tool is not used during the NuttX build, but
can be used as needed to generate files.

Usage:

.. code:: console

   $ ./mksymtab [-d] <cvs-file> <symtab-file> [<symtab-name> [<nsymbols-name>]]

Where::

    <cvs-file>      : The path to the input CSV file (required)
    <symtab-file>   : The path to the output symbol table file (required)
    <symtab-name>   : Optional name for the symbol table variable
                      Default: "g_symtab"
    <nsymbols-name> : Optional name for the symbol table variable
                      Default: "g_nsymbols"
    -d              : Enable debug output

Example:

.. code:: console

   $ cd nuttx/tools
   $ cat ../syscall/syscall.csv ../lib/libc.csv | sort >tmp.csv
   $ ./mksymtab.exe tmp.csv tmp.c
