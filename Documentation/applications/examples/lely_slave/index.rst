=========================================
``lely_slave`` Lely CANopen slave example
=========================================

A minimal CANopen **slave** node built on the `Lely CANopen
<https://gitlab.com/lely_industries/lely-core>`_ stack (see
:doc:`/applications/canutils/lely-canopen/index`).

The node is described by a static device description
(``sdev_slave.c``, node-ID ``0x02``) and talks to the CAN bus through the
NuttX **CAN character driver** (``/dev/can0``) by default, or through
SocketCAN when the ``CONFIG_EXAMPLES_LELYSLAVE_SOCKET`` backend is selected.

It registers the ``coslave`` NSH built-in command.

The slave answers the master's Client-SDO requests through its default
Server-SDO and, once the master has brought it to OPERATIONAL, streams an
incrementing counter (object ``0x2100``) in a synchronous TPDO (object
``0x1800``/``0x1a00``) on every SYNC.

Configuration
=============

``CONFIG_EXAMPLES_LELYSLAVE``
  Enable the example. Requires ``CANUTILS_LELYCANOPEN`` with ``SDEV``,
  ``TIME`` and ``OBJUPLOAD`` support.

``CONFIG_EXAMPLES_LELYSLAVE_CHARDEV`` / ``CONFIG_EXAMPLES_LELYSLAVE_SOCKET``
  CAN backend selection (character driver or SocketCAN).

``CONFIG_EXAMPLES_LELYSLAVE_DEVPATH``
  CAN character device path (default ``/dev/can0``).

``CONFIG_EXAMPLES_LELYSLAVE_INTF``
  SocketCAN interface name (default ``can0``).

The ready-made ``sim:lely`` configuration builds this example together with
the :doc:`/applications/examples/lely_master/index` demo for the simulator
with the CAN character driver enabled.
