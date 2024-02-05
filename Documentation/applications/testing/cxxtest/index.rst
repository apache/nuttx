============================
``cxxtest`` C++ test program
============================

This is a test of the C++ standard library. At present a port of the uClibc++
C++ library is available. Due to licensing issues, the uClibc++ C++ library is
not included in the NuttX source tree by default, but must be installed (see the
``README.txt`` file in the uClibc++ download package for installation).

The uClibc++ test includes simple test of:

- iostreams,
- STL,
- RTTI, and
- Exceptions

### Example Configuration Options

- ``CONFIG_TESTING_CXXTEST=y`` – Eanbles the example

### Other Required Configuration Settings

Other NuttX setting that are required include:

- ``CONFIG_HAVE_CXX=y``
- ``CONFIG_HAVE_CXXINITIALIZE=y``
- ``CONFIG_UCLIBCXX=y`` or ``CONFIG_LIBCXX=y``

Additional ``uClibc++/libcxx`` settings may be required in your build environment.
