===============
IMX95LPD5EVK-19
===============

The IMX95LPD5EVK-19 board is a platform designed to show the most commonly
used features of the
`i.MX 95 automotive applications processor
<https://www.nxp.com/products/iMX95>`_ .

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

Firmware location
=================

Instruction Tightly Coupled Memory (ITCM)
-----------------------------------------

The purpose of the Tightly-Coupled Memory (TCM) is to provide low-latency
memory that the processor can use without the unpredictability that is a
feature of caches. By default the firmware will be located in this area
(256K).

DDR
---

DDR memory can be used in case the code memory footprint becomes bigger than
the ITCM size. Using this configuration implies that other cores should be
aware of this.
For the default sd-card image from the EVK, these adaptations are needed on
the software running on the M33 and A55 cores.

    - `System Manager <https://github.com/nxp-imx/imx-sm>`_ (M33) should give
      the M7 access to the DDR region
    - `linux-imx <https://github.com/nxp-imx/linux-imx>`_ (A55) should reserve
      the DDR region by specifying it in the device tree so linux won't make
      use of it


Configurations
==============

All the configurations can be used in combination with the default sd-card
image that is shipped with the EVK.

nsh
---

Configures the NuttShell (nsh) located at examples/nsh.  This NSH
configuration is focused on low level, command-line driver testing. Built-in
applications are supported, but none are enabled. This configuration does not
support a network.


can
---

Configures the NuttShell (nsh) and also adds CAN support. CAN1 is enabled and
can be accessed at J17 on the EVK. Make sure that SW9[3] (PDM_CAN_SEL) is set
to ON. The configuration includes CAN utilities as candump and cansend.

.. note::
      `System Manager <https://github.com/nxp-imx/imx-sm>`_ (M33) should give
      the M7 access rights to the PIN_PDM_CLK (CAN1_TX) and
      PIN_PDM_BIT_STREAM0 (CAN1_RX) pins in the mx95evk.cfg.

      Alternatively these can be set manually in the system manager's console:

      .. code-block:: console

        >$ mm 0x443c01e0 0x6
        >$ mm 0x443c01e4 0x6


rpmsg
-----

This configuration is similar to nsh but in addition it offers the Remote
Processing Messaging (RPMsg) service to enable heterogeneous inter-core
communication. A virtual UART (CONFIG_RPMSG_UART) is made available on which
an OS running on the A55 cores can connect. There is also an option to use
the filesystem client feature in which a remote directory can be mounted to
a local directory (CONFIG_FS_RPMSGFS).

.. note::
      `linux-imx <https://github.com/nxp-imx/linux-imx>`_ (A55) needs the
      NuttX compatible rpmsg_tty and rpmsg_fs drivers. See `dev mailing list
      <https://www.mail-archive.com/dev@nuttx.apache.org/msg12112.html>`_