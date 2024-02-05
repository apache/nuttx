==============================
``helloxx`` Hello World in C++
==============================

This is C++ version of the "Hello, World" example. It is intended only to
verify that the C++ compiler is functional, that basic C++ library support is
available, and that class are instantiated correctly.

NuttX configuration prerequisites:

- ``CONFIG_HAVE_CXX`` – Enable C++ Support.

Optional NuttX configuration settings:

- ``CONFIG_HAVE_CXXINITIALIZE`` – Enable support for static constructors (may not
  be available on all platforms).

NuttX configuration settings specific to this example:

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the helloxx example as a built-in that can
  be executed from the NSH command line.

Also needed:

- ``CONFIG_HAVE_CXX=y``

And you may have to tinker with the following to get libxx to compile properly:

- ``CCONFIG_ARCH_SIZET_LONG=y`` or ``=n``.

The argument of the ``new`` operators should take a type of ``size_t``. But ``size_t``
has an unknown underlying. In the nuttx ``sys/types.h`` header file, ``size_t`` is
typed as ``uint32_t`` (which is determined by architecture-specific logic). But
the C++ compiler may believe that ``size_t`` is of a different type resulting in
compilation errors in the operator. Using the underlying integer type Instead of
``size_t`` seems to resolve the compilation issues.
