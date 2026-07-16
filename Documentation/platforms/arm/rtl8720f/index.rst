================
Realtek RTL8720F
================

The Realtek RTL8720F is a low-power multi-protocol wireless SoC from the
Realtek Ameba IoT family, combining 2.4 GHz Wi-Fi 6, Bluetooth LE and Thread
(IEEE 802.15.4) connectivity, and 2.4 GHz Wi-Fi radar sensing.

NuttX runs on the **KM4TZ application core** — an Arm Cortex-M55-compatible
core (Real-M300, Armv8.1-M) running at up to **320 MHz**, with a
single-precision FPU, DSP extensions and Arm TrustZone-M.

Highlights
==========

- **CPU:** Arm Cortex-M55-compatible application core, up to 320 MHz, with
  FPU + DSP instructions, TrustZone-M and instruction/data caches.
- **Memory:** 512 KB on-chip SRAM; external QSPI NOR flash (up to 104 MHz)
  and/or DDR PSRAM (up to 200 MHz), depending on the part number.
- **Wireless:** Wi-Fi 6 (802.11 b/g/n/ax), 2.4 GHz, up to 114.7 Mbps,
  WPA/WPA2/WPA3; Bluetooth LE 5.x; Thread (IEEE 802.15.4); and 2.4 GHz Wi-Fi
  radar sensing (CSI/RSSI).
- **Peripherals:** UART, SPI, I2C, I2S, SDIO, PWM, ADC, IR, GDMA, RTC and
  watchdogs (count varies by package); some I/O groups support 5 V levels.
- **Security:** Secure Boot, TrustZone-M, AES/SHA and ECDSA/RSA crypto
  engines, Flash decryption, OTP, and a true random number generator.
- **Package:** QFN40.

Memory
======

============ ============= =======
Block        Start Address Length
============ ============= =======
SRAM         0x2000_0000   512 KB
============ ============= =======

The on-chip SRAM holds the system heap and application. The flash is an SPI
NOR accessed through the SDK XIP path. The exact flash / PSRAM size depends on
the part number — for example the RTL8720FBF has 4 MB of NOR flash and no
PSRAM.

Vendor SDK and Toolchain
========================

The build depends on Realtek's open ``ameba-rtos`` SDK and its matching
``arm-none-eabi`` toolchain (from the Realtek asdk release), neither of which is
part of the NuttX tree.  The SDK provides the Wi-Fi / Bluetooth firmware and the
low-level chip libraries; NuttX links its own libc / libm and reuses the SDK's
``app_start()`` as the image entry point.

Both are fetched automatically — there is nothing to install by hand:

- **make** fetches them on the first ``make`` (from its ``PREBUILD`` step).
- **CMake** fetches them when you source ``. tools/ameba/env.sh <board>``, which
  must run before ``cmake`` (CMake probes the compiler at configure time).  The
  make build resolves everything on demand, so it needs no sourcing.

The SDK is a shallow ``git clone`` of the pinned revision of
``https://github.com/Ameba-AIoT/ameba-rtos.git`` into
``arch/arm/src/common/ameba/ameba-rtos`` (git-ignored) and is built unmodified;
export ``AMEBA_SDK`` to use a local checkout instead.  The asdk version is
pinned **per IC** by the SDK, so different Ameba ICs may use different toolchain
versions — the build selects the matching one automatically.

Building and Flashing
=====================

Replace ``<board>`` below with an actual board (e.g. ``rtl8720f_evb``) and
``<config>`` with one of its configurations.  The first build fetches the SDK
and toolchain (see `Vendor SDK and Toolchain`_).

With make
---------

.. code:: console

   $ ./tools/configure.sh <board>:<config>
   $ make

With CMake
----------

CMake probes the compiler at configure time, so source the Ameba environment
once first, **passing the board** so the asdk version that IC pins is on
``PATH``:

.. code:: console

   $ . tools/ameba/env.sh <board>
   $ cmake -B build -DBOARD_CONFIG=<board>:<config> -GNinja
   $ cmake --build build

make writes ``nuttx.bin`` to the top-level directory; CMake writes it under
``build/``.  The bootloader ``boot.bin`` is a prebuilt binary under the board's
``prebuilt/`` directory; the flash step writes both at the offsets taken from
the generated flash layout, so none are entered by hand.

Flashing
--------

**CLI (Linux/macOS)** — connect a USB-UART adapter and use the built-in flash
target (the baud defaults to 1500000; override with ``AMEBA_BAUD``)::

  $ make flash AMEBA_PORT=/dev/ttyUSB0                            # make build
  $ AMEBA_PORT=/dev/ttyUSB0 cmake --build build --target flash    # CMake build

**GUI (Windows)** — use the Realtek AmebaImageTool (``AmebaImageTool.exe`` under
``tools/ameba/ImageTool/`` in the SDK tree) to select ``boot.bin`` (from the
board's ``prebuilt/`` directory) and ``nuttx.bin``.  See the `Realtek Ameba
ImageTool guide <https://aiot.realmcu.com/en/latest/tools/image_tool/index.html>`_
for the Windows GUI tool and download-mode entry (hold the download button /
power-cycle with the ``UART_LOG_TX`` line asserted).

**Serial console** — after flashing, connect to the LOG-UART at 1500000 8N1::

  $ picocom -b 1500000 /dev/ttyUSB0

Configuration
=============

The build-time options are the same for make and CMake (both edit the one
Kconfig for the selected board); only the command that launches the menuconfig
UI differs:

.. code:: console

   $ make menuconfig                       # make build
   $ cmake --build build -t menuconfig     # CMake build

Supported Features
==================

- NSH over the LOG-UART console
- littlefs persistent storage at ``/data`` on the SPI NOR flash
- Wi-Fi station (scan / connect) and SoftAP via the ``wapi`` tool
- Networking on NuttX's own TCP/IP stack, with DHCP client and DHCP server

Boards
======

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
