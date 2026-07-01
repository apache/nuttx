================
Realtek RTL8720F
================

The Realtek RTL8720F is a dual-core Wi-Fi Host Controller (WHC) SoC, similar in
scheme to the :doc:`RTL8721Dx <../rtl8721dx/index>`:

- **KM4TZ** — an ARM Cortex-M33 (ARMv8-M.main, with FPU) application/host core
  running in the TrustZone secure world. NuttX runs here.
- **KM4NS** — the non-secure network-processor (NP) core that owns the Wi-Fi
  MAC/PHY and runs a prebuilt vendor firmware image.

NuttX is the Wi-Fi *host*: it talks to the NP over the on-chip IPC transport
(WHC) while the NP drives the radio. The IC-agnostic glue (os_wrapper backend,
netdev, key-value store, flash MTD, WHC Wi-Fi glue) is shared from
``arch/arm/src/common/ameba`` with the other Ameba WHC parts; only the
register-level drivers are IC-specific.

Memory Map
==========

============ ============= ======
Block Name   Start Address Length
============ ============= ======
SRAM         0x20000000    512K
============ ============= ======

The KM4TZ runs in the secure world, so the SRAM is also visible through the
secure alias at ``0x30000000``. The KM4TZ image2 RAM window is a slice of that
SRAM; the flash is a SPI NOR shared with the NP.

Vendor SDK Dependency
=====================

The build depends on Realtek's open ``ameba-rtos`` SDK, which is **not** part
of the NuttX tree. The first build auto-fetches the pinned revision (a shallow
``git clone`` of ``https://gitee.com/ameba-aiot/ameba-rtos.git``) into
``arch/arm/src/common/ameba/ameba-rtos`` (git-ignored) and applies the
out-of-SDK build patch under ``arch/arm/src/common/ameba/patches``. To use a
local checkout instead of auto-fetching, export ``AMEBA_SDK`` to its path.

A matching Realtek ARM toolchain (``arm-none-eabi`` from the Realtek asdk
release) is required; NuttX links its own libc/libm and reuses the SDK's
``app_start()`` as the image2 entry point.

Supported Features
==================

- NSH over the LOG-UART console (NuttX owns LOG-UART RX directly; the NP shell
  is disabled)
- littlefs persistent storage at ``/data`` on the SPI NOR
- Wi-Fi STA (scan / connect) and SoftAP via the ``wapi`` tool
- Networking over the WHC netdev (NuttX TCP/IP stack), DHCP client and server

Boards
======

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
