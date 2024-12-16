===============
IMX95LPD5EVK-19
===============

The IMX95LPD5EVK-19 board is a platform designed to show the most commonly
used features of the i.MX 95 automotive applications processor.

Features
========

- Multicore Processing [1]_
    - 1x Arm Cortex-M7
    - 6x Arm Cortex-A55 multicore complex
    - 1x Arm Cortex-M33
- Memory
    - On-Chip Memory
        - 1376kB SRAM (ECC)
    - External Memory
        - Up to 6.4GT/s x32 LPDDR5/LPDDR4X (with Inline ECC & Inline Encrpytion)
        - 1x Octal SPI, including support for SPI NOR and SPI NAND memories
- Connectivity
    - CAN FD
    - UART/USART/Profibus, I²C, SPI
    - Messaging Units (MU) to support IPC between heterogeneous cores

.. [1] NuttX is currently supported exclusively on the Cortex-M7 core of the
       i.MX95

Serial Console
==============

The IMX95LPD5EVK-19 board features a high-speed USB-to-UART/MPSSE device,
FT4232H (U70) that provides a debug interface for the i.MX95 processor through
the USB type-C connector (J31). The device acts as a bridge to enable
communication between the target processor and the host computer, which
connects to the USB connector (J31) through a USB cable.

Channel A is used as UART port to provide USB-to-UART option for debugging the
Arm Cortex-M7 core of the i.MX 95 processor (default option).

J-Link External Debug Probe
===========================

The IMX95LPD5EVK-19 board provides a 2x5-pin Samtec FTSH-105-01-L-DV-K header
(J30) for connecting a JTAG debugger (external JTAG) for debugging the i.MX95
processor. The FT4234H JTAG provides the remote debug option for the i.MX95
processor.

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at examples/nsh.  This NSH
configuration is focused on low level, command-line driver testing. Built-in
applications are supported, but none are enabled. This configuration does not
support a network.

This configuration can be used in combination with the default sd-card image
that is shipped with the EVK.

rpmsg
-----

This configuration is similar to nsh but in addition it offers the Remote
Processing Messaging (RPMsg) service to enable heterogeneous inter-core
communication. A virtual UART (CONFIG_RPMSG_UART) is made available on which
an OS running on the A55 cores can connect. There is also an option to use
the filesystem client feature in which a remote directory can be mounted to
a local directory (CONFIG_FS_RPMSGFS).

The rpmsg configuration executes the code from DDR since its code memory
footprint is bigger than the ITCM size. In the case of using the default
sd-card image from the EVK, adaptations are needed on the software running on
the M33 and A55 cores.

    - `System Manager <https://github.com/nxp-imx/imx-sm>`_ (M33) should give
      the M7 access to the DDR region
    - `linux-imx <https://github.com/nxp-imx/linux-imx>`_ (A55) should reserve
      the DDR region by specifying it in the device tree so linux won't make
      use of it
    - `linux-imx <https://github.com/nxp-imx/linux-imx>`_ (A55) needs the
      NuttX compatible rpmsg_tty and rpmsg_fs drivers. See `dev mailing list
      <https://www.mail-archive.com/dev@nuttx.apache.org/msg12112.html>`_