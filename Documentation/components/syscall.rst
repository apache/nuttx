=============
Syscall Layer
=============

This page discusses supports a syscall layer from communication between a
monolithic, kernel-mode NuttX kernel and a separately built, user-mode
application set.

With most MCUs, NuttX is built as a flat, single executable image
containing the NuttX RTOS along with all application code.  The RTOS code
and the application run in the same address space and at the same kernel-
mode privileges.  In order to exploit security features of certain
processors, an alternative build model is also supported:  NuttX can
be built separately as a monolithic, kernel-mode module and the applications
can be added as a separately built, user-mode module.

The syscall layer provided in this directory serves as the communication
layer from the user-mode application into the kernel-mode RTOS.  The
switch from user-mode to kernel-mode is accomplished using software
interrupts (SWIs).  SWIs are implemented differently and named differently
by different manufacturers but all work essentially the same:  A special
instruction is executed in user-mode that causes a software generated
interrupt.  The software generated interrupt is caught within the kernel
and handle in kernel-mode.

Header Files
------------

include/syscall.h
~~~~~~~~~~~~~~~~~

This header file supports general access to SWI facilities.  It is simply
a wrapper file that includes ``include/sys/syscall.h`` and
``include/arch/syscall.h``.

include/sys/syscall.h
~~~~~~~~~~~~~~~~~~~~~

The SWIs received by the kernel are distinguish by a code that identifies
how to process the SWI.  This header file defines all such codes understood
by the NuttX kernel.

include/arch/syscall.h (or arch/<cpu>/include/syscall.h)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This header file is provided by the platform-specific logic and declares
(or defines) the mechanism for providing software interrupts on this
platform.  The following functions must be declared (or defined) in this
header file:

- ``SWI`` with ``SYS_`` call number and one parameter

.. code-block:: C

    uintptr_t sys_call0(unsigned int nbr);

- ``SWI`` with ``SYS_`` call number and one parameter

.. code-block:: C

    uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1);

- ``SWI`` with ``SYS_`` call number and two parameters

.. code-block:: C

    uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1, uintptr_t parm2);

- ``SWI`` with ``SYS_`` call number and three parameters

.. code-block:: C

    uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                        uintptr_t parm2, uintptr_t parm3);

- ``SWI`` with ``SYS_`` call number and four parameters

.. code-block:: C

    uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1, uintptr_t parm2,
                        uintptr_t parm3, uintptr_t parm4);

- ``SWI`` with ``SYS_`` call number and five parameters

.. code-block:: C

    uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1, uintptr_t parm2,
                        uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);

- ``SWI`` with ``SYS_`` call number and six parameters

.. code-block:: C

    uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1, uintptr_t parm2,
                        uintptr_t parm3, uintptr_t parm4, uintptr_t parm5,
                        uintptr_t parm6);

Syscall Database
~~~~~~~~~~~~~~~~

Sycall information is maintained in a database.  That "database" is
implemented as a simple comma-separated-value file, ``syscall.csv``.  Most
spreadsheets programs will accept this format and can be used to maintain
the syscall database.

The format of the CSV file for each line is:

* Field 1: Function name

* Field 2: The header file that contains the function prototype

* Field 3: Condition for compilation

* Field 4: The type of function return value.

* Field 5 - N+5: The type of each of the N formal parameters of the function

* Fields N+5 - : If the last parameter is "...", then the following fields
  provide the type and number of of possible optional parameters.
  See note below about variadic functions

Each type field has a format as follows:

* type name:

  For all simpler types

* formal type | actual type:

  For array types where the form of the formal (eg. ``int parm[2]``)
  differs from the type of actual passed parameter (eg. ``int*``).
  This is necessary because you cannot do simple casts to array types.

* formal type | union member actual type | union member fieldname:

  A similar situation exists for unions.  For example, the formal
  parameter type union sigval -- You cannot cast a uintptr_t to
  a union sigval, but you can cast to the type of one of the union
  member types when passing the actual parameter.  Similarly, we
  cannot cast a union sigval to a uinptr_t either.  Rather, we need
  to cast a specific union member fieldname to ``uintptr_t``.

Variadic Functions
------------------

General variadic functions which may have an arbitrary number of argument
or arbitrary types cannot be represented as system calls.
``syslog()`` is a good example.   Normally you would work around this by
using the non- variadic form of the OS interface that accepts a ``va_list``
as an argument, ``vsyslog()`` in this case.

There there are many functions that have a variadic form but take only
one or two arguments optional arguments.  There can be handled as system
calls, but only by treating them as though they had a fixed number of
arguments.

These are are handled in ``syscall.csv`` by appending the number and type of
optional arguments.  For example, consider the ``open()`` OS interface.  Its
prototype is:

.. code-block:: C

      int open(const char *path, int oflag, ...);

In reality, open may take only a single optional argument of type ``mode_t``
and is represented in syscall.cvs like this::

      "open","fcntl.h","","int","const char*","int","...","mode_t"

The existence of the ``mode_t`` tells ``tools/mksyscall`` that there is at most
one optional parameter and, if present, it is of type ``mode_t``.

NOTE: This CSV file is used both to support the generate of trap information,
but also for the generation of symbol tables.  See ``Documentation/components/tools/``
and ``Documentation/components/libs/`` for further information.

Auto-Generated Files
--------------------

Stubs and proxies for the sycalls are automatically generated from this CSV
database.  Here the following definition is used:

* Proxy - A tiny bit of code that executes in the user space. A proxy
  has exactly the same function prototype as does the "real" function
  for which it proxies.  However, it only serves to map the function
  call into a syscall, marshaling all of the system call parameters
  as necessary.

* Stub  - Another tiny bit of code that executes within the NuttX kernel
  that is used to map a software interrupt received by the kernel to
  a kernel function call. The stubs receive the marshaled system
  call data, and perform the actually kernel function call (in
  kernel-mode) on behalf of the proxy function.

Sub-Directories
---------------

* ``stubs`` - Autogenerated stub files are placed in this directory.
* ``proxies`` - Autogenerated proxy files are placed in this directory.

mksyscall
---------

mksyscall is C program that is used used during the initial NuttX build
by the logic in the top-level ``syscall/``! directory. Information about the
stubs and proxies is maintained in a comma separated value (CSV) file
in the ``syscall/`` directory.  The mksyscall program will accept this CVS
file as input and generate all of the required proxy or stub files as
output.  See ``Documentation/components/tools/`` for additional information.
