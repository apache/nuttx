===
CRC
===

Overview
========

NuttX provides CRC (Cyclic Redundancy Check) implementations for CRC8, CRC16,
and CRC32 in ``libs/libc/misc/``. Each family offers multiple polynomial
variants, following a consistent naming convention.

CRC32 Variants
==============

NuttX ships several CRC32 variants that differ in polynomial, init value,
and final XOR. The two most commonly used are ``crc32()`` and ``crc32_ieee()``.

crc32() — NuttX Native
-----------------------

The original NuttX CRC32 implementation using the standard ISO 3309 / ITU-T
V.42 polynomial (0xEDB88320 reflected). It uses **init value 0** with
**no final XOR**:

.. code-block:: c

   uint32_t crc32(FAR const uint8_t *src, size_t len);
   uint32_t crc32part(FAR const uint8_t *src, size_t len, uint32_t crc32val);

This is the default CRC32 used by internal NuttX subsystems (bbsram, sbram,
etc.) and is maintained for backward compatibility.

crc32_ieee() — Linux/zlib Compatible
-------------------------------------

An IEEE-standard CRC32 compatible with Linux ``crc32()`` (from zlib). It uses
the same polynomial but with **init value 0xFFFFFFFF** and **final XOR
0xFFFFFFFF**:

.. code-block:: c

   uint32_t crc32_ieee(FAR const uint8_t *src, size_t len);
   uint32_t crc32_ieeepart(FAR const uint8_t *src, size_t len, uint32_t crc32val);

Use this variant when interoperating with Linux systems, e.g., for CRC
checksums in network protocols, file transfers, or UART communication layers.

Comparison
----------

==============  ============  ==========  ==========
Function        Init Value    Final XOR   Polynomial
==============  ============  ==========  ==========
crc32()         0x00000000    None        0xEDB88320
crc32_ieee()    0xFFFFFFFF    0xFFFFFFFF  0xEDB88320
==============  ============  ==========  ==========

Example: computing CRC32 of the string ``"123456789"``:

- ``crc32("123456789", 9)``      returns ``0x2DFD2D88``
- ``crc32_ieee("123456789", 9)`` returns ``0xCBF43926`` (matches Linux/zlib)

Incremental Calculation
-----------------------

Both variants support incremental (chunked) CRC calculation via the ``_part``
functions. For ``crc32_ieee``, the first call should pass ``0`` as the initial
value:

.. code-block:: c

   uint32_t crc = 0;
   crc = crc32_ieeepart(chunk1, len1, crc);
   crc = crc32_ieeepart(chunk2, len2, crc);
   /* crc now contains the final CRC of chunk1+chunk2 */

Other CRC32 Variants
--------------------

NuttX also provides CRC32 variants using different polynomials:

- ``crc32h04c11db7()`` / ``crc32h04c11db7_part()`` — Castagnoli (CRC-32C)
- ``crc32hf4acfb13()`` / ``crc32hf4acfb13_part()`` — CRC-32Q

CRC16 and CRC8
==============

CRC16 and CRC8 follow the same naming pattern with multiple variant functions.
See ``include/nuttx/crc16.h`` and ``include/nuttx/crc8.h`` for available
variants.
