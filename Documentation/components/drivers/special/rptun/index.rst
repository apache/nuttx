==========================
RPTUN (Remoteproc Tunnel)
==========================

RPTUN (Remoteproc Tunnel) is an efficient inter-core communication framework
based on OpenAMP. It primarily addresses the lifecycle management and data
exchange issues between the host core (running NuttX) and remote cores in
Asymmetric Multiprocessing (AMP) architectures.

The RPTUN framework consists of two core components:

- **RPTUN Remoteproc**: Manages the lifecycle of remote cores, including
  start, stop, and reset operations.

- **VirtIO/Vhost RPTUN**: Serves as the transport layer for VirtIO, enabling
  standard VirtIO/Vhost devices to communicate across physical cores.

Additionally, RPTUN exports standard character device interfaces to user space,
allowing developers to debug and control through command line or applications.

.. toctree::
   :maxdepth: 2

   architecture.rst
   resource_table.rst
   driver_porting.rst
   api_reference.rst

Target Audience
===============

This documentation is intended for embedded system engineers who need to
develop and port multi-core systems in NuttX environment, including:

- Application developers who need to use RPTUN for inter-core communication.
- Driver developers who need to adapt RPTUN on new hardware platforms (BSP).

Supported RPTUN Services
========================

- RPMSG File System
- RPMSG Domain (Remote) Sockets
- RPMSG UART Driver
- RPMSG Net Driver
- RPMSG Usersock
- RPMSG Sensor Driver
- RPMSG RTC Driver
- RPMSG MTD
- RPMSG Device
- RPMSG Block Driver
- RPMSG IO Expander
- RPMSG uinput
- RPMSG CLK Driver
- RPMSG Syslog
- RPMSG Regulator

Source Files
============

- Framework implementation: ``nuttx/drivers/rptun/rptun.c``
- Public header file: ``nuttx/include/nuttx/rptun/rptun.h``
