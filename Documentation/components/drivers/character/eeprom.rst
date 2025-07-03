======
EEPROM
======

.. warning::

   This page describes EEPROM interfacing using a character driver. For the
   more standard MTD interface, refer to the
   :doc:`MTD documentation <../special/mtd>`.
   See `MTD vs character driver <mtd_vs_char_>`_ for when to use each interface.

EEPROMs are a form of Memory Technology Device (MTD).

EEPROMs are non-volatile memory like FLASH, but differ in underlying memory
technology and differ in usage in many respects: They may not be organized into
blocks (at least from the standpoint of the user) and it is not necessary to
erase the EEPROM memory before re-writing it.

In addition, EEPROMs tend to be much smaller than FLASH parts, usually only a
few kilobytes vs megabytes for FLASH. EEPROM tends to be used to retain a small
amount of device configuration information; FLASH tends to be used for program
or massive data storage. For these reasons, it may not be convenient to use the
more complex MTD interface but instead use the simple character interface
provided by the EEPROM drivers.

.. _mtd_vs_char:

MTD driver vs character driver
==============================

MTD driver
  Used when the EEPROM should appear as a block device (`/dev/mtdX`) and is
  intended to be mounted with a filesystem. The MTD layer handles erase, read
  and write granularity.

Character driver
  Used when direct, random-access reads and writes to the raw EEPROM are
  desired, without the overhead of a filesystem. This is suitable for storing a
  handful of configuration parameters, calibration data, or any small blob that
  does not justify a full filesystem.

EEPROM Device Support
=====================

drivers/eeprom/spi_xx25xx.c
---------------------------

This is a driver for SPI EEPROMs that use the same commands as the
25AA160::

    Manufacturer Device     Bytes PgSize SecSize AddrLen
    Microchip
                 25xx010A     128     16      16       1
                 25xx020A     256     16      16       1
                 25AA02UID    256     16      16       1
                 25AA02E48    256     16      16       1
                 25AA02E64    256     16      16       1
                 25xx040      512     16      16       1+bit
                 25xx040A     512     16      16       1+bit
                 25xx080     1024     16      16       1
                 25xx080A    1024     16      16       2
                 25xx080B    1024     32      32       2
                 25xx080C    1024     16      16       x
                 25xx080D    1024     32      32       x
                 25xx160     2048     16      16       2
                 25xx160A/C  2048     16      16       2
                 25xx160B/D  2048     32      32       2
                 25xx160C    2048     16      16       2
                 25xx160D    2048     32      32       2
                 25xx320     4096     32      32       2
                 25xx320A    4096     32      32       2
                 25xx640     8192     32      32       2
                 25xx640A    8192     32      32       2
                 25xx128    16384     64      64       2
                 25xx256    32768     64      64       2
                 25xx512    65536    128   16384       2
                 25xx1024  131072    256   32768       3
    Atmel
                 AT25010B     128      8       8       1
                 AT25020B     256      8       8       1
                 AT25040B     512      8       8       1+bit
                 AT25080B    1024     32      32       2
                 AT25160B    2048     32      32       2
                 AT25320B    4096     32      32       2
                 AT25640B    8192     32      32       2
                 AT25128B   16384     64      64       2
                 AT25256B   32768     64      64       2
                 AT25512    65536    128     128       2
                 AT25M01   131072    256     256       3
    ST Microelectronics
                 M95010       128     16      16       1
                 M95020       256     16      16       1
                 M95040       512     16      16       1+bit
                 M95080      1024     32      32       2
                 M95160      2048     32      32       2
                 M95320      4096     32      32       2
                 M95640      8192     32      32       2
                 M95128     16384     64      64       2
                 M95256     32768     64      64       2
                 M95512     65536    128     128       2
                 M95M01    131072    256     256       3
                 M95M02    262144    256     256       3

drivers/eeprom/i2c_xx24xx.c
---------------------------

This is a driver for I2C EEPROMs that use the same commands as the xx24xx::

    Manufacturer Device     Bytes PgSize AddrLen DevAddr
    Microchip
                 24xx00        16     1    1     1010000 Special case
                 24xx01       128     8    1     1010000
                 24xx02       256     8    1     1010000
                 24xx04       512     16   1     101000P
                 24xx08      1024     16   1     10100PP
                 24xx16      2048     16   1     1010PPP
                 24xx32      4096     32   2     1010AAA
                 24xx64      8192     32   2     1010AAA
                 24xx128    16384     64   2     1010AAA
                 24xx256    32768     64   2     1010AAA
                 24xx512    65536    128   2     1010AAA
                 24xx1025  131072    128   2     1010PAA Special case: address
                                                         bit is shifted.
                 24xx1026  131072    128   2     1010AAP
    Atmel
                 AT24C01      128     8    1     1010AAA
                 AT24C02      256     8    1     1010AAA
                 AT24C04      512    16    1     1010AAP P bits = word address
                 AT24C08     1024    16    1     1010APP
                 AT24C16     2048    16    1     1010PPP
                 AT24C32     4096    32    2     1010AAA
                 AT24C64     8192    32    2     1010AAA
                 AT24C128   16384    64    2     10100AA
                 AT24C256   32768    64    2     10100AA
                 AT24C512   65536   128    2     10100AA
                 AT24C1024 131072   256    2     10100AP
    ST Microelectronics
                 M24C01       128    16    1     1010AAA
                 M24C02       256    16    1     1010AAA
                 M24C04       512    16    1     1010AAP
                 M24C08      1024    16    1     1010APP
                 M24C16      2048    16    1     1010PPP
                 M24C32      4096    32    2     1010AAA ID pages supported
                                                         as a separate device
                 M24C64      8192    32    2     1010AAA
                 M24128     16384    64    2     1010AAA
                 M24256     32768    64    2     1010AAA
                 M24512     65536   128    2     1010AAA
                 M24M01    131072   256    2     1010AAP
                 M24M02    262144   256    2     1010APP

IOCTL Commands
==============

The full list of ``ioctl()`` commands can be found in
``include/nuttx/eeprom/eeprom.h``.

-  ``EEPIOC_GEOMETRY``: Get the EEPROM geometry

File Systems
============

Most EEPROM parts are too small to be candidates for use with a file
system.  The character driver interface is optimal for these small parts
because you can open and access the EEPROM part as if it were a single,
fixed size file.

To use them with a file system, it is preferable to use the
:doc:`MTD driver <../special/mtd>`.
