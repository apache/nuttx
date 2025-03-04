=================
OS Drivers Design
=================

There are three kinds of drivers that are recognized by the OS and are visible to
applications. Two are POSIX standard device driver types, one is non-standard.
There are also internal OS components that may also be considered to be drivers
or, more correctly, lower-half drivers. Details about these are given below.

Character and Block Drivers
===========================

The standard driver types include:

* **Character Drivers**. First there are the character drivers These are drivers
  that support user accessibility via ``read()``, ``write()`` etc. The others do
  not naturally. Character drivers implement a stream of incoming or outgoing bytes.

* **Block Drivers**. These are used to support files systems that supported
  block-oriented I/O, not a character stream. The user cannot *directly* access
  block drivers.

The user can, however, access block drivers indirectly through a character driver proxy.
Both character and block drivers are represented by device nodes, usually in ``/dev``.
But if you try to open the block driver, something very strange happens: A temporary,
nameless proxy character driver is automatically instantiated that maps a character
driver's byte stream into blocks and mediates the driver access to the block driver.
This is the logic in ``drivers/bch``. BCH stands for block to character. So from the
application point of view, the both seem to be character drivers and applications
can interact with both in the same way.

This capability is exploited, for example, by the NuttX file system formatting
applications like mkfatfs to format a FAT system on a block driver.

There is also the complement, the loop device that converts a character driver into
a block driver. Loop devices are commonly used to format a file system image in RAM.

MTD Drivers
===========

And the non-standard driver is:

* The **Memory Technology Driver (MTD)**. This naming was borrowed from ``infradead.org``,
  but does not derive from any of their MTD logic. The MTD driver manages memory-based
  devices like FLASH or EEPROM. And MTD FLASH memory driver is very similar to a block
  driver but FLASH has some different properties, most notably that you have to erase
  FLASH before you write to it.

MTD has the same conveniences as block drivers: Then can appear as device nodes
under ``/dev`` and can be proxied to behave like character drivers if the opened
as character drivers. Plus they have some additional twists: MTD drivers can be
stacked one on top of another to extend the capabilities of the lower level MTD
driver. For example, ``drivers/mtd/sector512.c`` is an MTD driver that when layered
on top of another MTD driver, it changes the apparent page size of the FLASH to
512 bytes.

``drivers/mtd/mtd_partitions.c`` can be used to break up a large FLASH into
separate, independent partitions, each of which looks like another MTD driver.

``drivers/mtd/ftl.c`` is also interesting. FTL stands for FLASH Translation Layer.
The FTL driver is an MTD driver that when layered on top of another MTD driver,
converts the MTD driver to a block driver. The permutations are endless.

Monolithic Drivers
==================

When one thinks about device drivers in an OS, one thinks of a single thing,
a single block in a block diagram with these two primary interfaces:

* The device monolithic driver exposes a single, standard device driver interface.
  With the **Virtual File System (VFS)**, this provides the application user interface
  to the driver functionality. And

* A low-level interface to the hardware that is managed by the device driver.

Upper Half and Lower Half Drivers
=================================

NuttX supports many, many different MCU platforms, each with many similar but
distinct built-in peripherals.
Certainly we could imagine a realization where each such peripheral is supported
by monolithic driver as described in the preceding paragraph.
That would involve a lot code duplication, however.
The MCU peripherals may be unique at a low, register-level interface.
However, the peripherals are really very similar at a higher level of abstraction.

NuttX reduces the duplication, both in the code and in driver development,
using the notion of *Upper Half* and *Lower Half* drivers.
Such an implementation results in two things; two blocks in the system block
diagram: The upper half driver in a group of common, shared drivers, and
the MCU-specific lower half driver.

As before, each of these two driver components has two functional interfaces.
For the upper half driver:

* The upper half device driver exposes a single, standard driver interface.
  With the **Virtual File System (VFS)**, this, again, provides the application
  user interface to the driver functionality. And

* The upper-half side of the lower-half interface to the MCU-specific hardware
  that is managed by the lower-half device driver.

And for the lower half driver:

* The lower-half side of the interface to the the upper0half driver, and

* The low-level interface to the hardware that is managed by the lower half
  device driver.

One to Many: Encapsulation and Polymorphism
-------------------------------------------

These modular upper- and lower-half drivers have certain properties that you
would associate with an object oriented design: Encapsulation, data abstraction,
and polymorphism certainly.
Because of this encapsulation, the upper-half driver is complete unaware of any
implementation details within the lower-half driver.
Everything needed for the upper- and lower-half drivers to integrate is provided
by the defined interface between between those two things.
In fact, a single upper-half driver may service many lower-half driver instances
in a one-to-many relationship.

As an example, some MCUs support UARTs, USARTs functioning as UARTs,
Low-Power UARTs (LPUARTs), and other Flexible devices that may function as UARTs.
Each of these is managed by a separate lower-half driver that can be found in the
appropriate ``src/`` directory under ``arch/``.
In addition a board could have off-chip, external 16550 UART hardware (which has
a common lower-half driver).
Yet all of them would be supported by the single, common, serial upper half
driver that can be found at ``drivers/serial/serial.c``.
This is only possible due to the object-like properties of the lower-half driver
implementations.
