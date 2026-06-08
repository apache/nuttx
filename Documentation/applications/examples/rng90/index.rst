=============================
``rng90`` RNG90 TRNG Example
=============================

The ``rng90`` example is a simple NuttX application that opens an RNG90 device
node, reads random bytes, and prints them in hexadecimal format.

RNG90 is a dedicated hardware random number generator IC from Microchip. It can
be used as a companion device for microcontrollers that do not provide native
true random number generation (TRNG).

According to the Microchip RNG90 product page and linked validation artifacts,
the device is presented as a FIPS 140-3 compliant random number generator with
SP 800-90 A/B/C alignment, with references to NIST validation entries such as
entropy certificate E194 and DRBG validation A3013.

Microchip also positions this device for disposable/ecosystem-control
applications, which makes it a practical choice for cost-sensitive designs.

Configuration
=============

Enable this application in ``menuconfig``:

- ``Application Configuration -> Examples -> Microchip RNG90 TRNG example``

Required dependency:

- Enable the driver in ``menuconfig``:
  ``Device Drivers -> Cryptographic Device Drivers -> Enable Microchip RNG90 TRNG``
  (``CONFIG_DEV_RNG90``)

Application options:

- ``CONFIG_EXAMPLES_RNG90``
- ``CONFIG_EXAMPLES_RNG90_PROGNAME`` (default: ``"rng90"``)
- ``CONFIG_EXAMPLES_RNG90_DEVPATH`` (default: ``"/dev/rng0"``)
- ``CONFIG_EXAMPLES_RNG90_PRIORITY``
- ``CONFIG_EXAMPLES_RNG90_STACKSIZE``

Usage
=====

From NSH::

  rng90 [<devpath>] [<count>]

Arguments:

- ``<devpath>``: RNG90 device path (default from
  ``CONFIG_EXAMPLES_RNG90_DEVPATH``)
- ``<count>``: number of random bytes to read (``1..32``, default ``32``)

Examples::

  rng90
  rng90 /dev/rng0 16

Expected output
===============

::

  RNG90 example: device /dev/rng0
  Read 16 random byte(s):
  3f a1 7c ...

Notes
=====

- Opening the device wakes up RNG90.
- Closing the device puts RNG90 back to sleep.
- If ``<count>`` is outside ``1..32``, the app clamps it to ``32``.

Hardware validation log
=======================

The following session was executed on real hardware to validate this example.

Test setup:

- Host: macOS
- Board: ``esp32c3-devkit``
- Serial port: ``/dev/cu.wchusbserial140``
- Serial settings: ``115200 8N1``

Console transcript::

  nsh> uname -a
  NuttX 12.6.0-RC1 4130050287-dirty Jun  7 2026 22:11:21 risc-v esp32c3-devkit
  nsh> ls /dev/rng0
   /dev/rng0
  nsh> rng90
  RNG90 example: device /dev/rng0
  Read 32 random byte(s):
  0d b2 71 d1 40 1f 3c 50 52 3d c6 2c a9 74 ec 60
  44 ab 67 bb c4 34 1b ac 48 62 fe b9 fd 95 c9 cb
  nsh> rng90 /dev/rng0 16
  RNG90 example: device /dev/rng0
  Read 16 random byte(s):
  ef 82 5f b7 41 f1 48 13 5a 41 16 3a 67 1b 90 f2
  nsh> rng90 /dev/rng0 64
  Invalid count 64, clamping to 32
  RNG90 example: device /dev/rng0
  Read 32 random byte(s):
  bb be 2f 20 7f c8 33 8d 18 90 c8 fb 94 db bc e6
  84 41 17 6f 5a be 5a 06 c3 7b f1 30 ac f2 a6 f5
  nsh>

References
==========

- Microchip product page:
  https://www.microchip.com/en-us/product/RNG90
- NIST CMVP entropy certificate E194:
  https://csrc.nist.gov/projects/cryptographic-module-validation-program/entropy-validations/certificate/194
- NIST CAVP DRBG validation A3013:
  https://csrc.nist.gov/projects/cryptographic-algorithm-validation-program/details?validation=35623
