======================================
``nand`` - NAND Flash Device Simulator
======================================

In order to test the filesystems that work with NAND flash devices in a
simulator, this exists to provide a virtual NAND flash device, along with its
driver, to allow manual (or scripted) testing, as well as to provide an
option to log the various actions performed under-the-hood along with the
state of the device, which includes the read, write and erase counts of each
page in the device.

Structure of NAND Flash
=======================

Most NAND flashes share a common interface, specified by the
`Open NAND Flash Interface (ONFI) <https://www.onfi.org/>`_.

The important part from it, required in this context, is that a NAND Flash is
divided into a lot of blocks. And each blocks are divided into a lot of
pages.

Here's the peculiar bit. If you want to erase a page, you need to erase
the *entire* block that it is part of, ie. blocks are the smallest erasable
units in a NAND flash. However, a page is the smallest unit to which you can
write data, or read data from.

Why would you need erase operation? The Program/ Erase (P/E) cycle states
that a page (and thus its block) need to be erased first before data can be
written to it (Erase-Before-Write).

Each page has a data area, and a spare area. Depending on the data area's
size, the spare area may have different structures (schemes). All the
required schemes are defined in ``/drivers/mtd/mtd_nandscheme.c`` (in
the ``g_nand_sparescheme*`` structures).

Due to the nature of NAND flash, upon testing, a manufaturer may decide that
a certain block fails some test(s), and mark it as a **bad block** by
writing a certain value in a certain position in the spare area (depends on
data area's size, and thus, the spare area's scheme) of every page in it.

.. NOTE::
    * While certain blocks may *still work* even if they are marked as bad,
      it's inadvisable to store any data in it.

    * The spare data is the **only** record of a block being bad or not.
      Please do not erase it.

    * Certain blocks may become bad after continuous usage, and would need
      to be marked as such by either the filesystem or the driver.

Currently, this simulator supports only 512 B sized pages, which means it
will follow  the ``g_nand_sparescheme512`` scheme for its spare area, and
thus have a bad block marker at index ``5``.

If a block is *not* bad, it contains a value of ``0xff`` in the place of a
bad block marker. Any other value denote it's a bad block.

RAM to Device (Lower Half)
==========================

Since this is an emulation, RAM of the host running the simulator is used
to create the device. While the speed of operations won't be even close to
the original, this being in the RAM, which works multitudes faster than
actual device, the functionality on the other hand has been kept consistent
to the utmost.

First, ``/include/nuttx/mtd/nand.h`` has a structure ``struct nand_dev_s``
defining a raw NAND MTD device (lowest level). Its field ``nand_dev_s->raw``
is of type ``struct nand_raw_s *`` (defined in
``include/nuttx/mtd/nand_raw.h``), and this is what will hold the methods
for the raw device. There are primarily 3 methods that need to be looked
into:

* eraseblock
* rawread
* rawwrite

Conforming to the functionality of NAND flashes, these three were implemented
as ``nand_ram_*`` in ``/drivers/mtd/nand_ram.c`` to emulate RAM into a
virtual NAND flash.

While in real devices, the spare area follows the data area (in most schemes)
, since this is virtual, we can get away with keeping the two into two
separate arrays, namely ``nand_ram_flash_data`` and ``nand_ram_flash_spare``
for data and spare respectively. Each array has as many elements as number
of pages in the device.

As the spare areas has some spare bytes we can use, some space is used as
counters for the reads/writes/erases each page faces, thus giving a clear
picture of wear of the virtual device to the tester.

.. NOTE::
    ECC extension has not been implemented yet, so ECC bits in spare are
    yet to be used or initialized properly.

The method ``nand_ram_initialize()`` takes in a preallocated space for a
raw device (of type ``struct nand_raw_s`` as defined in
``include/nuttx/mtd/nand_raw.h``) and attaches these 3 custom methods as well
as device information like page size, block size, etc. to it. These form
the lower half of the driver.

Upper Half
==========

The method ``nand_ram_initialize()`` also initializes a
``struct mtd_dev_s *`` (defined in ``include/nuttx/mtd/mtd.h``), which it
returns. This structure contains a reference to our custom lower half in
``mtd_dev_s->raw``, as well as an upper half containing methods ``nand_*``
(defined in ``drivers/mtd/mtd_nand.c``).

Wrapper Over Upper Half
=======================

The upper half, along with the lower half attached to it, received from
``nand_ram_initialize()`` contains these 5 methods for the upper half:

* erase
* bread
* bwrite
* ioctl
* isbad
* markbad

Each driver's upper half needs to be registered with NuttX before it can
appear in the list of devices (in ``/dev``). Instead of the previously
acquired upper-half, we'll be registering a wrapper over it, to improve
logging. The wrapper methods are defined as ``nand_wrapper_*`` in
``drivers/mtd/mtd_nandwrapper.c``.

Here's a complicated bit. The actual upper half is an MTD device, but
more specifically, it is a NAND MTD device, which is represented by
``struct nand_dev_s``. Due to how it is defined, ``struct mtd_dev_s`` forms
the very start of ``struct nand_dev_s``, and hence they can be type-casted
to each other (provided required memory is accessible). Our wrapper, being a
wrapper over an MTD device, is an MTD device itself as well. MTD device
methods take in a ``struct mtd_dev_s *dev`` which describe the device
itself (which is the actual device that is registered using
``register_mtddriver``), which includes its methods. Our wrapper methods
receive such a device as well, which contains the wrapper device including
the wrapper functions. But, this way, there is no way of accessing the
methods of the actual upper half itself. Thus, instead of ``dev`` being
of type ``struct nand_dev_s``, it is actually of type
``struct nand_wrapper_dev_s`` which is a superset of ``struct nand_dev_s``,
who itself is a superset of ``struct mtd_dev_s``. Similar to previous case,
``struct mtd_dev_s`` forms the very start of ``struct nand_wrapper_dev_s``,
and thus the types are inter-changeable.

The methods ``nand_wrapper_*`` in ``drivers/mtd/mtd_nandwrapper.c`` all
pass the parameters to corresponding method of the actual upper half
after logging it. *But, the device passed on to the actual upper half
is still the wrapper, not the actual upper half, as the upper half methods
may call the other methods as well internally and we might want to log
them as well*.

Registering Device & Daemon
===========================

This wrapper is then registered using ``register_mtddriver``, and this
whole thing is converted to be a daemon, so that the device can keep running
in the background.

Making it a daemon is achieved by using ``fork()``, killing the parent, and
using ``daemon()`` in child.

Known Issues
============

* ECC is not implemented yet.
* MLC NAND Flash is not implemented yet.
* Due to the fixed name of the device, there can't be more than one instance
  of this virtual device.