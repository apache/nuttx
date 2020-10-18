==============
Device Drivers
==============

NuttX supports a variety of device drivers, which can be broadly
divided in three classes:

.. toctree::
  :maxdepth: 1

  character/index.rst
  block/index.rst
  special/index.rst

.. note::
  Device driver support depends on the *in-memory*, *pseudo*
  file system that is enabled by default.

Lower-half and upper-half
=========================

Drivers in NuttX generally work in two distinct layers:

  * An *upper half* which registers itself to NuttX using
    a call such as :c:func:`register_driver` or
    :c:func:`register_blockdriver` and implements the corresponding
    high-level interface (`read`, `write`, `close`, etc.).
    implements the interface. This *upper half* calls into
    the *lower half* via callbacks.
  * A "lower half" which is typically hardware-specific. This is
    usually implemented at the architecture or board level.

