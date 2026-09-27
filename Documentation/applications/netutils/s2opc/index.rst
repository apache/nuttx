================================
``s2opc`` OPC UA toolkit library
================================

Overview
========

S2OPC is an OPC UA toolkit from Systerel.  The NuttX package builds the OPC UA
client/server and UADP PubSub components with mbedTLS.  UDP, MQTT, and
raw-Ethernet PubSub transports are supported.  Optional XML loaders use Expat.
The package supports the NuttX Make and CMake build systems.

Configuration
=============

Enable ``CONFIG_NETUTILS_S2OPC``.  Its Kconfig dependencies require the
following facilities:

* mbedTLS
* IPv4 or IPv6 networking with TCP, UDP, socket options, and TCP keepalive
* socket binding to a named network device
* the network database interfaces
* POSIX recursive mutexes and a nonzero round-robin scheduler interval
* ``/dev/urandom`` backed by a suitable random-number generator

``CONFIG_S2OPC_THREAD_STACKSIZE`` controls the S2OPC worker-thread stack size.
Secure OPC UA processing needs significantly more stack than a small utility.

XML configuration is optional.  ``CONFIG_S2OPC_XML_LOADERS`` requires Expat and
builds the S2OPC client, server, user, NodeSet, and PubSub XML loaders.  Without
this option, applications configure PubSub through ``sopc_pubsub_conf.h`` and
configure client/server operation through the custom configuration APIs.
Address spaces can be generated as C source or constructed with the address
space API.  This avoids an XML parser and runtime XML files on constrained
targets.  ``CONFIG_ALLOW_MIT_COMPONENTS`` is required only when enabling the
Expat-backed XML loaders.

Optional client/server facilities are controlled by:

* ``CONFIG_S2OPC_NODE_MANAGEMENT`` for AddNodes and DeleteNodes
* ``CONFIG_S2OPC_NODE_ADD_OPTIONAL`` for optional type children
* ``CONFIG_S2OPC_HISTORY_READ`` for an application-provided raw-history store
* ``CONFIG_S2OPC_EVENT_MANAGEMENT`` for server events
* ``CONFIG_S2OPC_AUDITING`` for S2OPC 1.7.3 security audit events

The application must load a mutable address space before using node
management.  History reads require an application callback and history store.
Event management requires the corresponding event types in the address space.

PubSub transports
=================

UDP is always included with ``CONFIG_NETUTILS_S2OPC``.  UDP multicast requires
the corresponding NuttX IGMP or MLD configuration.  A named multicast
interface requires ``CONFIG_NETDEV_IFINDEX``.  Binding a socket to a named
device requires ``CONFIG_NET_BINDTODEVICE``.

``CONFIG_S2OPC_MQTT`` enables the Eclipse Paho MQTT Async backend and requires
``CONFIG_LIB_MQTT5``.  The NuttX Paho package uses its high-performance mode to
avoid host-oriented heap and call-stack tracing tables.  This does not disable
MQTT protocol operation.

``CONFIG_S2OPC_ETHERNET`` enables UADP over NuttX ``AF_PACKET`` sockets and
requires ``CONFIG_NET_PKT``, packet-protocol socket options, and multicast
group support through IGMP, MLD, or ICMPv6.  The network driver must implement
multicast MAC filtering.  The NuttX backend requires an interface name and,
for multicast reception, a destination multicast MAC address.  A process that
opens simultaneous publisher and subscriber packet sockets needs at least two
packet connections, for example
``CONFIG_NET_PKT_PREALLOC_CONNS=2``.

See :doc:`/applications/examples/s2opc/index` for the OPC UA server example.
