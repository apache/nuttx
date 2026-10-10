==================================
``brctl`` Ethernet bridge command
==================================

Overview
========

The ``brctl`` command creates and deletes Ethernet bridges and adds or
removes their ports, using the ``SIOCBRADDBR``, ``SIOCBRDELBR``,
``SIOCBRADDIF`` and ``SIOCBRDELIF`` ioctl commands.  See
:doc:`/os/networking/bridge` for how the bridge works.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_BRCTL``.  It depends on
``CONFIG_NET_BRIDGE``.

Usage
=====

.. code-block:: console

   brctl addbr <bridge>
   brctl delbr <bridge>
   brctl addif <bridge> <device>
   brctl delif <bridge> <device>

``addbr``
  Create a bridge device named ``<bridge>``.
``delbr``
  Delete a bridge.  The bridge must be down (``ifdown <bridge>``).
``addif``
  Add the network device ``<device>`` to the bridge as a port.
``delif``
  Remove a port from the bridge.

Example
=======

.. code-block:: console

   nsh> ifup eth0
   nsh> ifup eth1
   nsh> brctl addbr br0
   nsh> brctl addif br0 eth0
   nsh> brctl addif br0 eth1
   nsh> ifconfig br0 10.0.0.2 netmask 255.255.255.0
   nsh> ifup br0
