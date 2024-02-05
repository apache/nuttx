===============
NuttX libraries
===============

This page discusses the NuttX libraries that can be found in ``libs/``

Libraries in NuttX are very special creatures.  They have these properties:

#. They can be shared by both application logic and logic within the OS when
   using the FLAT build.

#. But in PROTECTED and KERNEL modes, they must be built differently:  The
   copies used by applications and the OS cannot be the same.  Rather,
   separate versions of libraries must be built for the kernel and for
   applications.

#. When used by the OS, some special care must be taken to assure that the
   OS logic does not disrupt the user's errno value and that the OS does
   not create inappropriate cancellation points.

   For example, ``sem_wait()`` is both a cancellation point and modifies the
   errno value.  So within the FLAT build and without kernel version for
   the PROTECTED and KERNEL builds, the special internal OS interface
   ``nxsem_wait()`` must be used.

NOTE:  The libraries under ``libs/`` build differently from other NuttX
components:  There are no build-related files in the ``libs/`` directory; it
is simply a container for other well-known, individual library directories.
The upper level Makefile logic is aware of the libraries within the ``libs/``
container.

The only real function of the ``libs/`` directory is to prevent the top-level
directory from becoming cluttered with individual libraries.

.. toctree::
   :maxdepth: 1
   :caption: Contents:
   
   libc/index.rst
   libdsp.rst
   libm.rst
   libxx.rst
   libnx/index.rst
