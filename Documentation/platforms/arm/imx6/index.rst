===================
NXP/Freescale i.MX6
===================

The basic port has been completed for the following i.MX6 board:

-  **Sabre-6Quad**. This is a port to the NXP/Freescale Sabre-6Quad
   board. Refer to the NuttX board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/imx6/sabre-6quad/README.txt>`__
   file for further information.

   **STATUS:** The basic, minimal port is code complete and introduced
   in NuttX-7.15, but had not yet been tested at that time due to the
   inavailability of hardware. This basic port was verified in the
   NuttX-7.16 release, however. The port is still minimal and more
   device drivers are needed to make the port usable.

   Basic support of NuttX running in SMP mode on the i.MX6Q was also
   accomplished in NuttX-7.16. However, there are still known issues
   with SMP support on this platform as described in the
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/imx6/sabre-6quad/README.txt>`__
   file for the board.
