=====
eFuse
=====

An eFuse is memory that can be written **once**.  A bit that has been burned
stays burned: there is no erase, and no way back.  Chips use them for the
things that must not change after manufacture -- a serial number, a MAC
address, a secure boot key, a flag that permanently disables the debug port.

Enabled with ``CONFIG_EFUSE``.  The code is in ``drivers/efuse/``, and the
interface is in ``include/nuttx/efuse/efuse.h``.

Interface
=========

A chip port registers a device with

.. code-block:: c

   FAR void *efuse_register(FAR const char *path,
                            FAR struct efuse_lowerhalf_s *lower);

conventionally at ``/dev/efuse``, and releases it with
``efuse_unregister()``.  Ports exist for the ESP32 family, the ESP32-C3, the
SAMA5 and the RP2350's OTP.

Everything then happens through ``ioctl()`` on that path:

``EFUSEIOC_READ_FIELD``
   Read a blob of bits out of a field.

``EFUSEIOC_WRITE_FIELD``
   Write a blob of bits into a field.

``EFUSEIOC_MASK``
   Mask the fuse registers so they can no longer be read.  Only the
   ATSAMA5D2 and ATSAMA5D4 use it.

``read()`` and ``write()`` on the device are **stubs that return zero**.
Nothing is read and nothing is written, and because zero from ``read()``
ordinarily means end of file, a program that reaches for them gets silence
rather than an error.

A field is not a name.  The argument to the two field ioctls carries an
array of

.. code-block:: c

   struct efuse_desc_s
   {
     uint16_t   bit_offset; /* Bit offset related to beginning of efuse */
     uint16_t   bit_count;  /* Length of bit field */
   };

so the caller says where the bits are and how many, and the upper half hands
that straight to the lower half without looking at it.  The upper half is
thin on purpose: what a fuse *means* is entirely chip specific.  Names, where
they exist at all, are there for the caller and not for the driver:
``include/nuttx/efuse/sama5_sfc_fuses.h`` lists the SAMA5 fuses as an enum,
and no code in the tree reads it.

The lower half supplies ``read_field()``, ``write_field()`` and an
``ioctl()`` that receives any command the upper half does not recognise.

Handle with care
================

A fuse is one-time programmable: writing the wrong value is permanent, and
no reboot undoes it.  On many parts one of the available fuses disables the
debug interface, so a mistake there costs you the board, not just the boot.

Two consequences worth designing around:

* Read fuses freely; write them from as little code as possible, ideally
  from a dedicated provisioning program that is not part of the shipping
  firmware.
* Check what a field means in the chip's own documentation before writing
  it.  The NuttX driver will happily burn whatever it is told to.
