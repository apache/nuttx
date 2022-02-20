.. _binfmt:

=============
Binary Loader
=============

The purpose of a *binary loader* is to load and
execute modules in various *binary formats* that reside in a file
system. Loading refers instantiating the binary module in some fashion,
usually copy all or some of the binary module into memory and then
linking the module with other components. In most architectures, it is
the base FLASH code that is the primary component that the binary module
must link with because that is where the RTOS and primary tasks reside.
Program modules can then be executed after they have been loaded.

**Binary Formats**. The binary loader provides generic support for
different binary formats. It supports a *registration interface* that
allows the number of support binary formats to be loaded at run time.
Each binary format provides a common, interface for use by the binary
loader. When asked to load a binary, the binary loader will query each
registered binary format, providing it with the path of the binary
object to be loaded. The binary loader will stop when first binary
format the recognizes the binary object and successfully loads it or
when all registered binary formats have attempt loading the binary
object and failed.

At present, the following binary formats are support by NuttX:

  - **ELF**. Standard ELF formatted files.
  - **NXFLAT**. NuttX NXFLAT formatted files. More information about the
    NXFLAT binary format can be found in the :ref:`NXFLAT
    documentation <nxflat>`.

**Executables and Libraries** The generic binary loader logic does not
care what it is that it being loaded. It could load an executable
program or a library. There are no strict rules, but a library will tend
to export symbols and a program will tend to import symbols: The program
will use the symbols exported by the library. However, at this point in
time, none of the supported binary formats support exporting of symbols.

**binfmt**. In the NuttX source code, the short name ``binfmt`` is used
to refer to the NuttX binary loader. This is the name of the directory
containing the binary loader and the name of the header files and
variables used by the binary loader.

The name ``binfmt`` is the same name used by the Linux binary loader.
However, the NuttX binary loader is an independent development and
shares nothing with the Linux binary loader other the same name and the
same basic functionality.

Binary Loader Interface
=======================

Header Files
------------

The interface to the binary loader is described in the header file
``include/nuttx/binfmt/binfmt.h``.
A brief summary of the data structurs and interfaces prototyped in that
header file are listed below.

Data Structures
---------------

When a binary format registers with the binary loader, it provides a
pointer to a write-able instance of :c:struct:`binfmt_s`.

.. c:struct:: binfmt_s

  .. code-block:: c

    struct binfmt_s
    {
      FAR struct binfmt_s *next;             /* Supports a singly-linked list */
      int (*load)(FAR struct binary_s *bin); /* Verify and load binary into memory */
    };

  The ``load`` method is used to load the binary format into memory. It
  returns either ``OK`` (0) meaning that the binary object was loaded
  successfully, or a negated ``errno`` indicating why the object was not
  loaded.

.. c:struct:: binary_s

  The type ``struct binary_s`` is use both to (1) describe the binary
  object to be loaded, and if successfully loaded, (2) to provide
  information about where and how the binary object was loaded. That
  structure is shown below:

  .. code-block:: c

    struct symtab_s;
    struct binary_s
    {
      /* Information provided to the loader to load and bind a module */

      FAR const char *filename;            /* Full path to the binary to be loaded */
      FAR const char **argv;               /* Argument list */
      FAR const struct symtab_s *exports;  /* Table of exported symbols */
      int nexports;                        /* The number of symbols in exports[] */

      /* Information provided from the loader (if successful) describing the
       * resources used by the loaded module.
       */

      main_t entrypt;                      /* Entry point into a program module */
      FAR void *mapped;                    /* Memory-mapped, address space */
      FAR void *alloc[BINFMT_NALLOC];      /* Allocated address spaces */

      /* Constructors/destructors */

    #ifdef CONFIG_BINFMT_CONSTRUCTORS
      FAR binfmt_ctor_t *ctors;            /* Pointer to a list of constructors */
      FAR binfmt_dtor_t *dtors;            /* Pointer to a list of destructors */
      uint16_t nctors;                     /* Number of constructors in the list */
      uint16_t ndtors;                     /* Number of destructors in the list */
    #endif

      /* Address environment.
       *
       * addrenv - This is the handle created by up_addrenv_create() that can be
       *   used to manage the tasks address space.
       */

    #ifdef CONFIG_ARCH_ADDRENV
      group_addrenv_t addrenv;             /* Task group address environment */
    #endif

      size_t mapsize;                      /* Size of the mapped address region (needed for munmap) */

      /* Start-up information that is provided by the loader, but may be modified
       * by the caller between load_module() and exec_module() calls.
       */

      uint8_t priority;                    /* Task execution priority */
      size_t stacksize;                    /* Size of the stack in bytes (unallocated) */
    };

  Where the types ``binfmt_ctor_t`` and ``binfmt_dtor_t`` define the type
  of one C++ constructor or destructor:

  .. code-block:: c

    typedef FAR void (*binfmt_ctor_t)(void);
    typedef FAR void (*binfmt_dtor_t)(void);

Function Interfaces
-------------------

Binary format management
~~~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int register_binfmt(FAR struct binfmt_s *binfmt)

  Register a loader for a binary format.

  :return: This is a NuttX internal function so it follows the convention
    that 0 (OK) is returned on success and a negated errno is returned on
    failure.

.. c:function:: int unregister_binfmt(FAR struct binfmt_s *binfmt)

  Register a loader for a binary format.

  :return:
    This is a NuttX internal function so it follows the convention
    that 0 (OK) is returned on success and a negated errno is returned on
    failure.

Basic module management
~~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: int load_module(FAR struct binary_s *bin)

  Load a module into memory, bind it to an exported symbol take,
  and prep the module for execution.

  :param bin:
    The ``filename`` field will be used
    in order to locate the module to be loaded from the file system.
    The filename must be the full, absolute path to the file to be executed
    unless ``CONFIG_LIBC_ENVPATH`` is defined. In that case, filename may be
    a relative path; a set of candidate absolute paths will be generated using
    the ``PATH`` environment variable and ``load_module()`` will attempt to load each
    file that is found at those absolute paths.

  :return:
    This is a NuttX internal function so it follows the convention that 0 (``OK``)
    is returned on success and a negated ``errno`` is returned on failure.

.. c:function:: int unload_module(FAR struct binary_s *bin)

  Unload a (non-executing) module from memory. If the module has been started
  (via :c:func:`exec_module`) and has not exited, calling this will be fatal.

  However, this function must be called after the module exist. How this is
  done is up to your logic. Perhaps you register it to be called by :c:func:`on_exit`?

  :return:
    This is a NuttX internal function so it follows the convention that 0 (``OK``)
    is returned on success and a negated ``errno`` is returned on failure.

.. c:function:: int exec_module(FAR const struct binary_s *bin);

  Execute a module that has been loaded into memory by :c:func:`load_module`.

  :return:
    This is a NuttX internal function so it follows the convention that 0 (``OK``)
    is returned on success and a negated ``errno`` is returned on failure.

.. tip::
  The function :c:func:`exec` is a convenience function that wraps
  :c:func:`load_module` and :c:func:`exec_module` into one call.

``PATH`` traversal logic
~~~~~~~~~~~~~~~~~~~~~~~~

.. c:function:: ENVPATH_HANDLE envpath_init(void);

  Initialize for the traversal of each value in the ``PATH`` variable. The
  usage is sequence is as follows:

  #. Call :c:func:`envpath_init` to initialize for the traversal.
     ``envpath_init()`` will return an opaque handle that can then be
     provided to :c:func:`envpath_next` and :c:func:`envpath_release`.
  #. Call :c:func:`envpath_next` repeatedly to examine every file that lies in
     the directories of the ``PATH`` variable.
  #. Call :c:func:`envpath_release` to free resources set aside by
     :c:func:`envpath_init`.

  :return:
    On success, :c:func:`envpath_init` return a non-``NULL``, opaque handle
    that may subsequently be used in calls to :c:func:`envpath_next` and
    :c:func:`envpath_release`. On error, a ``NULL`` handle value will be returned.
    The most likely cause of an error would be that there is no value
    associated with the ``PATH`` variable.

.. c:function:: FAR char *envpath_next(ENVPATH_HANDLE handle, FAR const char *relpath)

  Traverse all possible values in the PATH variable in attempt to find the
  full path to an executable file when only a relative path is provided.

  :param handle: The handle value returned by :c:func:`envpath_init`.
  :param relpath: The relative path to the file to be found.

  :return:
    On success, a non-``NULL`` pointer to a null-terminated string is provided.
    This is the full path to a file that exists in the file system.
    This function will verify that the file exists (but will not verify that it is marked executable).

  .. note::
    The string pointer return in the success case points to allocated memory.
    This memory must be freed by the called by calling :c:func:`kmm_free`.

  ``NULL`` relpath from any absolute path in the ``PATH`` variable.
  In this case, there is no point in calling :c:func:`envpath_next` further;
  :c:func:`envpath_release` must be called to release resources set aside by
  :c:func:`envpath_init`.

.. c:function:: void envpath_release(ENVPATH_HANDLE handle)

Release all resources set aside by envpath_init when the
handle value was created. The handle value is invalid on
return from this function. Attempts to all :c:func:`envpath_next`
or :c:func:`envpath_release` with such a stale handle will result
in undefined (i.e., not good) behavior.

  :param handle: The handle value returned by :c:func:`envpath_init`.

Symbol Tables
=============

**Symbol Tables**. Symbol tables are lists of name value mappings: The
name is a string that identifies a symbol, and the value is an address
in memory where the symbol of that name has been positioned. In most
NuttX architectures symbol tables are required, as a minimum, in order
to dynamically link the loaded binary object with the base code on
FLASH. Since the binary object was separately built and separately
linked, these symbols will appear as *undefined* symbols in the binary
object. The binary loader will use the symbol table to look up the
symbol by its name and to provide the address associated with the symbol
as needed to perform the dynamic linking of the binary object to the
base FLASH code.

Some toolchains will prefix symbols with an underscore. To support these
toolchains the ``CONFIG_SYMTAB_DECORATED`` setting may be defined. This
will cause a leading underscore to be ignored on *undefined* symbols
during dynamic linking.

Symbol Table Header Files
-------------------------

The interface to the symbol table logic is described in the header file
``include/nuttx/binfmt/symtab.h``.
A brief summary of the data structurs and interfaces prototyped in that
header file are listed below.

Symbol Table Data Structures
----------------------------

.. c:struct:: symbtab_s

  Describes one entry in the symbol table.

  .. code-block:: c

    struct symtab_s
    {
      FAR const char *sym_name;          /* A pointer to the symbol name string */
      FAR const void *sym_value;         /* The value associated with the string */
    };

  A symbol table is a fixed size array of ``struct symtab_s``. The
  information is intentionally minimal and supports only:

  #. Function pointers as ``sym_values``. Of other kinds of values need to
     be supported, then typing information would also need to be included
     in the structure.
  #. Fixed size arrays. There is no explicit provisional for dynamically
     adding or removing entries from the symbol table (realloc might be
     used for that purpose if needed). The intention is to support only
     fixed size arrays completely defined at compilation or link time.

Symbol Table Function Interfaces
--------------------------------

.. c:function:: FAR const struct symtab_s *symtab_findbyname(FAR const struct symtab_s *symtab, FAR const char *name, int nsyms);

  Find the symbol in the symbol table with the matching name.
  The implementation will be linear with respect to ``nsyms`` if
  ``CONFIG_SYMTAB_ORDEREDBYNAME`` is not selected, and logarithmic
  if it is.

  :return:
    A reference to the symbol table entry if an entry with
    the matching name is found; NULL is returned if the entry is not found.

.. c:function:: FAR const struct symtab_s *symtab_findbyvalue(FAR const struct symtab_s *symtab, FAR void *value, int nsyms);

  Find the symbol in the symbol table whose value closest
  (but not greater than), the provided value. This version assumes
  that table is not ordered with respect to symbol value and, hence,
  access time will be linear with respect to ``nsyms``.

  :return:
    A reference to the symbol table entry if an entry with the matching
    value is found; ``NULL`` is returned if the entry is not found.

Configuration Variables
=======================

  - ``CONFIG_BINFMT_DISABLE``: By default, support for loadable binary formats is built.
    This logic may be suppressed be defining this setting.
  - ``CONFIG_BINFMT_CONSTRUCTORS``: Build in support for C++ constructors in loaded modules.
  - ``CONFIG_SYMTAB_ORDEREDBYNAME``: Symbol tables are order by name (rather than value).
  - ``CONFIG_SYMTAB_DECORATED``: Symbols will have a leading underscore in object files.

Additional configuration options may be required for the each enabled
binary format.
