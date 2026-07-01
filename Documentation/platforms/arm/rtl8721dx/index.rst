=================
Realtek RTL8721Dx
=================

The Realtek RTL8721Dx is a dual-core Wi-Fi Host Controller (WHC) SoC:

- **KM4** — an ARM Cortex-M33 (ARMv8-M.main, with FPU) application/host core.
  NuttX runs here.
- **KM0** — the network-processor (NP) core that owns the Wi-Fi MAC/PHY and
  runs a prebuilt vendor firmware image.

NuttX is the Wi-Fi *host*: it talks to the NP over the on-chip IPC transport
(WHC, ``WHC_HOST`` + ``WHC_INTF_IPC``) while the NP drives the radio. The
IC-agnostic glue (os_wrapper backend, netdev, key-value store, flash MTD, WHC
Wi-Fi glue) is shared from ``arch/arm/src/common/ameba`` with the other Ameba
WHC parts; only the register-level drivers are IC-specific.

Memory Map
==========

============ ============= ======
Block Name   Start Address Length
============ ============= ======
SRAM (KM4)   0x20020000    288K
============ ============= ======

The KM4 image2 RAM window is a slice of the on-chip SRAM; the flash is a SPI
NOR accessed via the SDK XIP path and shared with the NP.

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

- NSH over the LOG-UART console
- littlefs persistent storage at ``/data`` on the SPI NOR (a dedicated flash
  partition), backing the Wi-Fi key-value store
- Wi-Fi STA (scan / connect) and SoftAP via the ``wapi`` tool
- Networking (lwIP-free: NuttX's own TCP/IP stack over the WHC netdev),
  DHCP client and DHCP server

Boards
======

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
