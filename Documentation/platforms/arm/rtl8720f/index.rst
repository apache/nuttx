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

Vendor SDK Dependency
=====================

The build depends on Realtek's open ``ameba-rtos`` SDK, which is **not** part
of the NuttX tree. It provides the Wi-Fi / Bluetooth firmware and the
low-level chip libraries. The first build auto-fetches the pinned revision (a
shallow ``git clone`` of ``https://github.com/Ameba-AIoT/ameba-rtos.git``) into
``arch/arm/src/common/ameba/ameba-rtos`` (git-ignored) and builds against it
unmodified. To use a local checkout instead of auto-fetching, export
``AMEBA_SDK`` to its path.

Toolchain
=========

A matching Realtek ARM toolchain (``arm-none-eabi`` from the Realtek asdk
release) is required and is fetched automatically by the board build. NuttX
links its own libc / libm and reuses the SDK's ``app_start()`` as the image
entry point.

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
