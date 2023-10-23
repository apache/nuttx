===============================
``libtest`` Static Library Test
===============================

This example illustrates how you may create a static library. It does the
following:

It creates a static library called libtest.a that contains an object that provides
the symbol library_test().

At adds the library as an EXTRA_LIB in the build::

  EXTRA_LIBS += -ltest
  EXTRA_LIBPATHS += -L$(APPDIR)/examples/libtest

And optionally, it can be configured to:

Generate a built-in command that can be executed by NSH. This command logic links
with the symbol library_test() that will provided by the libtest.a static library.
