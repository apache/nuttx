=================
Realtek RTL8721Dx
=================

The Realtek RTL8721Dx is a low-power dual-band Wi-Fi and Bluetooth LE SoC from
the Realtek Ameba IoT family, targeting smart-home control, wireless audio,
battery-powered cameras, smart locks and other IoT products.

NuttX runs on the **KM4 application core** — an Arm Cortex-M55-compatible core
(Real-M300, Armv8.1-M) running at up to **345 MHz**, with a single-precision
FPU, DSP extensions and Arm TrustZone-M.

Highlights
==========

- **CPU:** Arm Cortex-M55-compatible application core, up to 345 MHz, with
  FPU + DSP instructions, TrustZone-M, and 16 KB I-Cache + 16 KB D-Cache.
- **Memory:** 512 KB on-chip SRAM; external QSPI NOR flash (up to 104 MHz)
  and/or DDR PSRAM (up to 200 MHz), depending on the part number.
- **Wireless:** Wi-Fi 4 (802.11 a/b/g/n), dual-band 2.4 GHz / 5 GHz, 1T1R, up
  to 150 Mbps, WPA/WPA2/WPA3; Bluetooth LE 5.0.
- **Peripherals:** UART x4, I2C x2, SPI x2, QSPI/OSPI, I2S x2, DMIC x2,
  PWM x8, ADC + cap-touch, USB 2.0 full-speed device, SDIO device, IR, RTC,
  independent/system watchdogs, GDMA, and up to 64 GPIO (count varies by
  package).
- **Security:** Secure Boot, TrustZone-M, AES/SHA hardware crypto engine, OTP,
  and a true random number generator (PSA Level 2).
- **Packages:** QFN48, QFN68 and BGA100.

Memory
======

============ ============= =======
Block        Start Address Length
============ ============= =======
SRAM         0x2000_0000   512 KB
============ ============= =======

The on-chip SRAM holds the system heap and application. The flash is an SPI
NOR accessed through the SDK XIP path. The exact flash / PSRAM size depends on
the part number — for example the RTL8721DAF has 4 MB of NOR flash and no
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
- littlefs persistent storage at ``/data`` on the SPI NOR flash (a dedicated
  partition), backing the Wi-Fi key-value store
- Wi-Fi station (scan / connect) and SoftAP via the ``wapi`` tool
- Networking on NuttX's own TCP/IP stack, with DHCP client and DHCP server

Boards
======

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
