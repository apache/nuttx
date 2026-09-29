====================
Auxiliary lwIP Stack
====================

Overview
========

``CONFIG_NET_LWIP`` builds lwIP 2.0.3 as an auxiliary TCP/IP stack alongside
the native NuttX network stack.  It does not replace NuttX socket routing.
Applications select lwIP explicitly through its prefixed API, such as
``lwip_socket()``, ``lwip_connect()``, and ``lwip_send()``.

The initial integration supports the simulator Ethernet driver.  Both stacks
share the same simulated Ethernet device through an L2 tap.  The lwIP
interface therefore uses ``10.0.1.3`` by default while the native NuttX
interface uses ``10.0.1.2``.

Configuration
=============

Use the simulator configuration as a starting point::

  ./tools/configure.sh sim:lwip
  make

The main options are under ``Networking Support -> lwIP Auxiliary Stack``.
DHCP is disabled by default because the native and auxiliary stacks share one
link.  The static address, network mask, and gateway are derived from the
``CONFIG_NETINIT_*`` values.

Initialization
==============

Call ``nuttx_lwip_initialize()`` once from a long-lived initialization
context before using the lwIP API.  The call is idempotent and creates the
lwIP TCP/IP and receive threads.  In a protected or kernel build, place the
call in an equivalent persistent system service.

The lifetime requirement is important on NuttX: initializing from a
short-lived application associates the worker threads with that
application's task group, so they are reclaimed when the application exits.

Simulator Network
=================

The simulator must have a TAP interface whose host address is
``10.0.1.1/24``.  Once the interface and a host receiver are ready, UDP and
TCP traffic can be sent from lwIP applications to the host.  A continuous
test should run UDP, TCP, and throughput cases in the same NSH session; this
also verifies that the lwIP worker threads remain alive between application
invocations.
