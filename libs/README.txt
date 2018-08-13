README
======

This directory holds NuttX libraries.  Libraries in NuttX are very special
creatures.  The have these properties:

1. They can be shared by both application logic and logic within the OS when
   using the FLAT build.

2. But in PROTECTED and KERNEL modes, they must be built differently:  The
   copies used by applications and the OS cannot be the same.  Rather,
   separate versions of libraries must be built for the kernel and for
   applications.

3. When used by the OS, some special care must be taken to assure that the
   OS logic does not disrupt the user's errno value and that the OS does
   not create inappropriate cancellation points.

   For example, sem_wait() is both a cancellation point and modifies the
   errno value.  So within the FLAT build and without kernel version for
   the PROTECTED and KERNEL builds, the special internal OS interface
   nxsem_wait() must be used.  Within libraries, the macro  _SEM_WAIT()
   (as defined in include/nuttx/semaphore.h) is used instead.  The
   definition of this macro accounts for the different usage environments.

NOTE:  The libraries under libs/ build differently from other NuttX
components:  There are no build-related files in the libs/ directory; it
is simply a container for other well-known, individual library directories.
The upper level Makefile logic is aware of the libraries within the libs/
container.

The only real function of the libs/ directory is to prevent the top-level
directory from becoming cluttered with individual libraries.
