===============
``gencromfs.c``
===============

This is a C program that is used to generate CROMFS file system images.
Usage is simple:

.. code:: console

   $ gencromfs <dir-path> <out-file>

Where:

* ``<dir-path>`` is the path to the directory will be at the root of the new
  CROMFS file system image.

* ``<out-file>`` the name of the generated, output C file. This file must be
  compiled in order to generate the binary CROMFS file system image.
