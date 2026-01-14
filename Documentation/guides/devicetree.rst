====================
Device Tree
====================

Overview
--------

Currently, NuttX supports parsing of FDT(Flattened Device Tree) using libfdt, a
utility library for reading and manipulating the binary format:

https://github.com/dgibson/dtc/

Based on that, NuttX has implemented some common functions to get properties.
Device Tree support in NuttX will reduce the configuration of chips/boards,
it is not used in NuttX kernel framework yet.

How to use
-----------

1. Enable Device Tree and libfdt

Enable the Kconfig settings:

    .. code-block:: console

      CONFIG_DEVICE_TREE=y                        /* Enable Device Tree */
      CONFIG_LIBFDT=y                             /* Enable utility library */

2. Register the DTB address

Use fdt_register to set the DTB Address in NuttX

3. Parse the DTB

Chip/board will use fdt_get to get the DTB Address, and then use fdt_* APIs to
parse the DTB properties.
