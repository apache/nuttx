====================
``cmocka`` libcmocka
====================

cmocka is:
    an elegant unit testing framework for C with support for mock objects. It
    only requires the standard C library, works on a range of computing
    platforms (including embedded) and with different compilers.

The cmocka application in the NuttX apps is an adaptation of cmocka into the
NuttX build system.

You can find out more about how to use the cmocka library on the official
`cmocka website <https://cmocka.org/index.html>`_.

Interface Notes
---------------

The cmocka library header requires that several other standard header files be
included prior to it. Your include statement should look like this:

.. code:: c

   #include <setjmp.h>
   #include <stdarg.h>
   #include <stddef.h>
   #include <stdint.h>

   #include <cmocka.h>

Build options
-------------

* ``CONFIG_TESTING_CMOCKA_PROG``: If enabled, a `cmocka` test binary will be
  included in your image.

* ``CONFIG_TESTING_CMOCKA_LEAKDETECT``: Cmocka can check for memory leaks.
  Intended to be used in a local environment.
