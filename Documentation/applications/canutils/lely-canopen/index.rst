=====================================
``lely-canopen`` Lely CANopen support
=====================================

`Lely CANopen <https://gitlab.com/lely_industries/lely-core>`_ is an
open-source CANopen (CiA 301/302/305/315) protocol stack.

The port configuration (``LELY_NO_*`` feature switches) is generated
from Kconfig in ``apps/include/canutils/lely/config.h``.

Feature selection
=================

``CONFIG_CANUTILS_LELYCANOPEN`` enables the build. The individual CANopen
services are selected with the ``CONFIG_CANUTILS_LELYCANOPEN_*`` options,
including:

- ``SDEV`` – static device description support (selects ``OBJNAME``)
- ``MASTER`` – NMT master support
- ``EMCY`` – emergency (EMCY) objects
- ``CSDO`` / ``SSDO`` – client / server SDO
- ``RPDO`` / ``TPDO`` / ``MPDO`` – process data objects
- ``SYNC`` / ``TIME`` – SYNC and TIME producers/consumers
- ``LSS`` – layer setting services
- ``NG`` / ``NMTBOOT`` / ``NMTCFG`` – node guarding and master boot/config
- ``OBJDEFAULT`` / ``OBJLIMITS`` / ``OBJNAME`` / ``OBJUPLOAD`` /
  ``OBJFILE`` – object dictionary features

Tools
=====

``CONFIG_CANUTILS_LELYCANOPEN_TOOLS_COCTL`` builds ``coctl``, the Lely
CANopen control tool (an interactive ASCII gateway). It requires SocketCAN
(``NET_CAN``).

Examples
========

Two ready-to-run demos built on this library are provided:

- :doc:`/applications/examples/lely_slave/index` – a CANopen slave node
- :doc:`/applications/examples/lely_master/index` – a CANopen master node

Both can run on the simulator over the CAN character driver. See
:ref:`testing_lely_canopen_sim` for how to set up a virtual CAN network and
observe the traffic.
