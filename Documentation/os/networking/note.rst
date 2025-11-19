================
Note over NETDEV
================

Note over NETDEV streams kernel tracing or instrumentation data (e.g., scheduler
events, interrupts, context switches) to a remote host for analysis.
It is one-way communication over UDP.

The netdev driver selected by ``CONFIG_DRIVERS_NOTENET_INTF`` must be based on
the :ref:`Netdev Lowerhalf <netdriver>` implementation, and ``CONFIG_NET_NETPOLL``
must be enabled. Notes are sent as raw note records to
``CONFIG_DRIVERS_NOTENET_IPADDR`` / ``CONFIG_DRIVERS_NOTENET_MAC`` on UDP port
``CONFIG_DRIVERS_NOTENET_PORT`` (broadcast by default).
