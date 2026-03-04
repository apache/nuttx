=================================================
``mksyscall.c``, ``cvsparser.c``, ``cvsparser.h``
=================================================

This is a C file that is used to build mksyscall program.  The mksyscall
program is used during the initial NuttX build by the logic in the top-
level syscall/ directory.

If you build NuttX as a separately compiled, monolithic kernel and separate
applications, then there is a syscall layer that is used to get from the
user application space to the NuttX kernel space.  In the user application
"proxies" for each of the kernel functions are provided.  The proxies have
the same function signature as the kernel function, but only execute a
system call.

Within the kernel, there are "stubs" for each of the system calls.  The
stubs receive the marshalled system call data, and perform the actually
kernel function call (in kernel-mode) on behalf of the proxy function.

Information about the stubs and proxies is maintained in a comma separated
value (CSV) file in the syscall/ directory.  The mksyscall program will
accept this CVS file as input and generate all of the required proxy or
stub files as output.  See :doc:`/components/syscall` for additional information.

