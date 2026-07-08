===========================================
``lely_master`` Lely CANopen master example
===========================================

A minimal CANopen **master** node built on the `Lely CANopen
<https://gitlab.com/lely_industries/lely-core>`_ stack (see
:doc:`/applications/canutils/lely-canopen/index`).

The node is described by a static device description
(``sdev_master.c``, node-ID ``0x01``) and talks to the CAN bus through the
NuttX **CAN character driver** (``/dev/can0``) by default, or through
SocketCAN when the ``CONFIG_EXAMPLES_LELYMASTER_SOCKET`` backend is selected.

It registers the ``comaster`` NSH built-in command.

Once the slave has booted, the master drives a small CANopen network:

#. reads the slave's device type (object ``0x1000``) with a Client-SDO
   upload;
#. commands the slave to OPERATIONAL with an NMT start;
#. produces SYNC (object ``0x1005``/``0x1006``) and receives the slave's
   synchronous TPDO through an RPDO (object ``0x1400``/``0x1600``), printing
   the streamed counter that lands in object ``0x2100``.

Configuration
=============

``CONFIG_EXAMPLES_LELYMASTER``
  Enable the example. Requires ``CANUTILS_LELYCANOPEN`` with ``SDEV``,
  ``TIME``, ``OBJUPLOAD`` and ``MASTER`` support (the master services also
  pull in ``EMCY``, ``CSDO``, ``RPDO``, ``TPDO`` and ``SYNC``).

``CONFIG_EXAMPLES_LELYMASTER_CHARDEV`` / ``CONFIG_EXAMPLES_LELYMASTER_SOCKET``
  CAN backend selection (character driver or SocketCAN).

``CONFIG_EXAMPLES_LELYMASTER_DEVPATH``
  CAN character device path (default ``/dev/can0``).

``CONFIG_EXAMPLES_LELYMASTER_INTF``
  SocketCAN interface name (default ``can0``).

The ready-made ``sim:lely`` configuration builds this example together with
the :doc:`/applications/examples/lely_slave/index` demo for the simulator
with the CAN character driver enabled.
