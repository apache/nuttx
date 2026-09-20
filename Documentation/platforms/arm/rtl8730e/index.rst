================
Realtek RTL8730E
================

The Realtek RTL8730E is a high-performance, multi-core IoT SoC from the
Realtek Ameba family, targeting smart-display panels, automotive HMI,
video doorbells, industrial tablets and edge-AI gateways.

NuttX runs on the **dual-core Arm Cortex-A32 application processor** at up to
**1.2 GHz**. The KM4 network core and the KM0 low-power core continue to run
the vendor wireless firmware independently; NuttX interacts with them through
the WHC (Wireless Host Controller) IPC interface.

Highlights
==========

- **CPU:** Dual-core Arm Cortex-A32 (Armv8-A ISA, AArch32), up to 1.2 GHz,
  with NEON/FPU and hardware virtualisation extensions.
- **Co-processors:** Real-M300 (KM4, Cortex-M55-compatible, 333 MHz) for
  networking; Real-M200 (KM0, Cortex-M23-compatible, 40 MHz) for power
  management.
- **Memory:** 256 KB on-chip SRAM; up to 256 MB DDR3L or 64 MB DDR2 external
  DRAM; 8/16/32 MB SPI NOR flash (varies by part).
- **Wireless:** Wi-Fi 6 (802.11ax), dual-band 2.4 GHz / 5 GHz, 1T1R; Bluetooth
  5 dual-mode (BR/EDR + LE) with LE Audio / Auracast support.
- **Peripherals:** UART ×4, SPI ×2, I2C ×3, I2S ×2, PDM, USB 2.0, MIPI-DSI,
  SD/eMMC, PWM, ADC, RTC, watchdogs, GDMA, and 38–59 GPIO (package-dependent).
- **Security:** Secure Boot, Arm TrustZone-A, AES/SHA/RSA/ECDSA hardware
  crypto, OTF flash decryption, TRNG, OTP, PSA Level 3 / FIPS 140-3.
- **Packages:** QFN100 (10×10 mm), DR-QFN144 (11×11 mm).
- **Temperature:** −40 °C to 105 °C.

Boot Chain
==========

The RTL8730E uses Arm Trusted Firmware (ATF).  NuttX is packaged as BL33
inside a Firmware Image Package (FIP) alongside the vendor BL2 and BL32
(SP_MIN).  The flash layout is::

  boot.bin  @ 0x08000000  — KM4 bootloader
  app.bin   @ 0x0803FC00  — KM0+KM4 firmware + ATF FIP (BL2+BL32+BL33=NuttX)

The ``make flash`` target assembles the FIP and app.bin automatically; there
is nothing to prepare by hand (see `Building and Flashing`_ below).

Memory
======

============ ============= =========
Block        Start Address Length
============ ============= =========
DDR          0x6000\_0000  64 MB
============ ============= =========

NuttX executes entirely from DDR (loaded by ATF from NOR flash at boot).
The exact DDR size depends on the part; the RTL8730EH has 64 MB DDR2.
NuttX is loaded at 0x60300000 with the heap starting at 0x60400000.  The
256 KB on-chip SRAM is partitioned among the KM4 and KM0 co-processors and
is not directly used by the CA32 NuttX image.

Vendor SDK and Toolchain
========================

The build depends on Realtek's open ``ameba-rtos`` SDK (Wi-Fi / BT firmware
and low-level chip libraries) and the matching ``arm-none-eabi`` asdk
toolchain.  Neither is part of the NuttX tree; both are fetched automatically:

- **make** fetches them on the first ``make`` invocation (from its ``PREBUILD``
  step).
- **CMake** fetches them when you source ``. tools/ameba/env.sh <board>``
  before running ``cmake``.

The SDK is a shallow ``git clone`` of the pinned revision of
``https://github.com/Ameba-AIoT/ameba-rtos.git`` into
``arch/arm/src/common/ameba/ameba-rtos`` (git-ignored) and is built
unmodified.  Export ``AMEBA_SDK`` to use a local checkout instead.

Building and Flashing
=====================

Replace ``<board>`` with an actual board (e.g. ``rtl8730e_evb``) and ``<config>``
with one of its configurations.  The first build fetches the SDK and toolchain.

With make
---------

.. code:: console

   $ ./tools/configure.sh <board>:<config>
   $ make

With CMake
----------

Source the Ameba environment once first so the correct asdk is on ``PATH``:

.. code:: console

   $ . tools/ameba/env.sh <board>
   $ cmake -B build -DBOARD_CONFIG=<board>:<config> -GNinja
   $ cmake --build build

Flashing
--------

Connect a USB-UART adapter and use the built-in flash target (baud defaults
to 1500000; override with ``AMEBA_BAUD``)::

  $ make flash AMEBA_PORT=/dev/ttyUSB0                            # make build
  $ AMEBA_PORT=/dev/ttyUSB0 cmake --build build --target flash    # CMake build

The flash target automatically:

1. Packages ``nuttx.bin`` as BL33 into a FIP together with the prebuilt
   ``bl2.bin`` and ``bl32.bin``.
2. Prepends the vendor KM0/KM4 firmware prefix to produce ``app.bin``.
3. Writes ``boot.bin`` and ``app.bin`` to the NOR flash via
   ``AmebaFlash.py`` using the ``RTL8730E_NOR.rdev`` profile.

**Serial console** — after flashing, connect to the LOG-UART at 1500000 8N1::

  $ picocom -b 1500000 /dev/ttyUSB0

Configuration
=============

.. code:: console

   $ make menuconfig                       # make build
   $ cmake --build build -t menuconfig     # CMake build

Supported Features
==================

- NSH over the LOG-UART console
- Symmetric Multi-Processing (SMP) on both Cortex-A32 cores
- littlefs persistent storage at ``/data`` on the SPI NOR flash, backing the
  Wi-Fi key-value store
- Wi-Fi station (scan / connect) and SoftAP via the ``wapi`` tool
- Networking on NuttX's own TCP/IP stack, with DHCP client and DHCP server
- ``/etc`` read-only ROMFS populated from the board's ``src/etc/`` tree

Boards
======

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
