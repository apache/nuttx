============
TI AM62x
============

`TI AM62x <https://www.ti.com/product/AM6254>`_ is a family of ARM64 SoCs from
Texas Instruments featuring:

- **CPU:** Quad-Core ARM Cortex-A53 (up to 1.4 GHz)
- **Interrupt Controller:** ARM GIC-600 (GICv3), 480 SPIs
- **Boot:** ROM → R5 SYSFW (TIFS) → U-Boot SPL → U-Boot → NuttX
- **Memory:** External DDR4/LPDDR4 (512 MB – 2 GB depending on board)
- **UART:** Seven 16550-compatible UARTs (48 MHz clock)
- **GPIO:** Main-domain GPIO controller support
- **I2C:** Main-domain I2C controller support

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
