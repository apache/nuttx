===================
SDIO Device Drivers
===================

-  ``include/nuttx/sdio.h``. All structures and APIs needed to
   work with SDIO drivers are provided in this header file.

-  ``struct sdio_dev_s``. Each SDIO device driver must
   implement an instance of ``struct sdio_dev_s``. That structure
   defines a call table with the following methods:

   Mutual exclusion:

   Initialization/setup:

   Command/Status/Data Transfer:

   Event/Callback support:

   DMA support:

-  **Binding SDIO Drivers**. SDIO drivers are not normally
   directly accessed by user code, but are usually bound to
   another, higher level device driver. In general, the binding
   sequence is:

   #. Get an instance of ``struct sdio_dev_s`` from the
      hardware-specific SDIO device driver, and
   #. Provide that instance to the initialization method of the
      higher level device driver.

-  **Examples**: ``arch/arm/src/stm32/stm32_sdio.c`` and
   ``drivers/mmcsd/mmcsd_sdio.c``

Implementing an SDIO lower-half
===============================

When implementing a new SDMMC controller driver (SDIO lower-half), it must
provide the interface defined in ``struct sdio_dev_s``.

Call-flow (simplified example)
------------------------------

The full SDIO/MMCSD call-flow for card identification and initialization
is more complex and includes additional commands (e.g., CMD0, CMD8,
ACMD41 / CMD1, CMD2, CMD3, error handling, retries, etc.).  For the
purposes of documenting the R2/CID/CSD handling expected from the
lower-half, a simplified interaction around CMD9 looks like this:

1. ``SDIO_SENDCMD``: Send the command that yields an R2 response
   (e.g., CMD2 for CID, CMD9 for CSD).
2. ``SDIO_WAITRESPONSE``: Poll for the hardware to complete the command.
3. ``SDIO_RECVR2``: Retrieve the 136-bit response and provide the
   decoded 128-bit CID/CSD payload to the MMCSD upper-half.

R2 (136-bit) response and CSD/CID
---------------------------------

The standard R2 response format includes a 7-bit CRC that many hardware
controllers automatically verify and strip. The MMCSD upper-half expects the
provided 128-bit buffer to contain the CID or CSD payload in its standard
layout (bits 127-0).

If the controller strips the CRC byte, the remaining bits in the hardware
registers are often misaligned (shifted). The lower-half MUST shift the four
32-bit words left by one byte (8 bits) before returning them via ``recv_r2``
if the CRC is not included in the registers.

Refer to ``arch/arm64/src/bcm2711/bcm2711_sdio.c`` or
``arch/arm64/src/imx9/imx9_usdhc.c`` for reference implementations of this
shifting logic.
