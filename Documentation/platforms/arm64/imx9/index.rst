=========
NXP i.MX9
=========

Peripheral Support
==================

==========  =======  =====================================================
Peripheral  Support  Notes
==========  =======  =====================================================
ELE         Yes      EdgeLock Enclave, used for entropy and secure boot
RNG         Yes      ``/dev/random`` and ``/dev/urandom``, backed by the ELE
==========  =======  =====================================================

RNG
---

The EdgeLock Enclave contains a true random number generator. ``CONFIG_IMX9_RNG``
registers it as ``/dev/random`` and ``/dev/urandom``, which is what seeds the
kernel entropy pool from hardware. Without it ``up_randompool_initialize()``
has no hardware source.

``CONFIG_IMX9_RNG`` selects ``CONFIG_IMX9_ELE`` and ``CONFIG_ARCH_HAVE_RNG``.
``CONFIG_DEV_RANDOM`` then defaults on; ``CONFIG_DEV_URANDOM`` does not and must
be set explicitly, after which ``CONFIG_DEV_URANDOM_ARCH`` selects the hardware
backend rather than the software PRNG.

A read is served in blocks, looping until the request is satisfied. Every block
is checked before it is returned, and a failed check is an error rather than a
short read: the buffer must have been written by the ELE, must not be all
zeros, and must differ from the previous block, which is the FIPS 140-2
continuous test.

.. code-block:: console

   nsh> hexdump /dev/random -c 32
   /dev/random at 00000000:
   0000: 7c 9a f9 b0 fb 89 be 9d 23 bf 17 28 88 43 67 10
   0010: c5 79 10 9e d7 be ca 42 2f 51 00 bd bc ce 77 54

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
