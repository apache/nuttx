===================================
Block-to-character (BCH) conversion
===================================

Block drivers are read and written a sector at a time; character drivers
are read and written a byte at a time.  BCH sits between the two, so a block
device can be opened, read and written as if it were a character device.

The code is in ``drivers/bch/``.

What it is for
==============

Two situations, both common:

* A program wants to read a few bytes from the middle of a partition --
  a header, a serial number, a configuration blob -- and does not want to
  know the sector size.
* A file system image has to be written to a partition from user space, with
  an ordinary ``write()``, rather than a sector at a time.

You rarely register one yourself
================================

Opening a block device by name already goes through BCH.  ``open()`` sees
that the inode is a block driver -- or an MTD driver -- and hands it to
``block_proxy()`` in ``fs/driver/fs_blockproxy.c``, which registers a BCH
character device for it under a temporary name, ``/dev/tmpcNNNNNN``, opens
that, and unlinks the name again.  So reading a few bytes out of
``/dev/mtdblock0`` with ``read()`` needs no registration at all.

``bchdev_register()`` is for when you want a lasting name instead of a
temporary one.

``CONFIG_BCH_DEVICE_READONLY`` forces every one of those proxy opens to
read-only.

Interface
=========

.. c:function:: int bchdev_register(FAR const char *blkdev, FAR const char *chardev, int oflags)

   Exports the block driver at ``blkdev`` as the character device
   ``chardev``.

   :param blkdev: The block device that already exists, for example
     ``/dev/mtdblock0``.
   :param chardev: The character device to create.  Pick a path nothing else
     claims: ``/dev/mtdN``, for instance, is where ``register_mtddriver()``
     puts MTD drivers, so it is a poor choice here.
   :param oflags: Open flags.  Read-only unless write access is asked for.
     Asking for write access fails with ``-EACCES`` if the block driver has
     no write method or reports that writing is disabled.
   :return: Zero on success; a negated ``errno`` on failure.

.. c:function:: int bchdev_unregister(FAR const char *chardev)

   Removes a character device created by ``bchdev_register()``.

   :param chardev: The character device to remove.
   :return: Zero on success; a negated ``errno`` on failure.

How it works, and what it costs
===============================

BCH keeps one sector-sized cache, and uses it only for the parts of a
transfer that are not a whole sector.  A transfer that starts or ends in the
middle of a sector passes that end through the cache; for a write that means
a read-modify-write, because the sector has to be read before the bytes in
it can be changed, and is written back afterwards.  Whole sectors in between
skip the cache entirely and are passed to the block driver from, or into,
the caller's own buffer.

That is the cost worth knowing.  Writing one byte through BCH reads and
rewrites a whole sector, and on flash a sector rewrite may mean an erase, so
a program that writes bytes one at a time will be slow and will wear the
device.  Writing whole sectors, aligned, avoids both the read and the copy.

``CONFIG_BCH_FORCE_INDIRECT`` takes the direct path away and sends every
transfer through the cache.  Some configurations need it -- the Kconfig
names ``CONFIG_BUILD_KERNEL`` as the case -- because there the block driver
cannot be handed the caller's buffer to write from.

``CONFIG_BCH_BUFFER_ALIGNMENT`` is the alignment the cache is allocated
with, through ``kmm_memalign()``.  The default, zero, asks for no particular
alignment.

Encryption
==========

``CONFIG_BCH_ENCRYPTION`` encrypts the sector as it passes through, using a
key of ``CONFIG_BCH_ENCRYPTION_KEY_SIZE`` bytes.  It requires
``CONFIG_CRYPTO_AES``.

It is not plain AES over the sector.  Each 16-byte block is encrypted with
XEX -- xor, encrypt, xor -- and the value that is xored in is built from the
sector number together with the block's index inside the sector, then itself
encrypted with AES-ECB.  Deriving it from the sector number is what stops
two identical blocks in different sectors from encrypting to the same
ciphertext.  The loop covers ``sectsize / 16`` blocks, so a sector size that
is not a multiple of 16 would leave a tail unencrypted.
