=====================
``dhcpc`` DHCP Client
=====================

The ``dhcpc`` utility is a lightweight Dynamic Host Configuration Protocol (DHCP)
client for Apache NuttX, implementing the client protocol defined in :rfc:`2131`
with BOOTP relay interoperability per :rfc:`1542`.

It provides embedded systems with automated IPv4 address acquisition, subnet mask
assignment, default gateway discovery, Domain Name System (DNS) resolver
configuration, and Network Time Protocol (NTP) server discovery over local Ethernet
or wireless (Wi-Fi) interfaces.

.. note::

   **Security Consideration**: Standard DHCPv4 (:rfc:`2131`) transmits messages in
   plaintext without cryptographic authentication. In untrusted or shared network
   environments, malicious actors may deploy rogue DHCP servers to inject spoofed
   default gateways or malicious DNS resolvers (enabling Man-in-the-Middle attacks)
   or conduct DHCP starvation attacks. On managed networks, switch-level protections
   (such as DHCP Snooping and Port Security) should be enabled. When operating on
   untrusted links, secure transport protocols (e.g., TLS, IPsec, or WireGuard) and
   authenticated DNS (DNS-over-TLS / DNSSEC) should be employed to protect application
   data regardless of the assigned network parameters.

Operational Roles
=================

The DHCP client library serves multiple key functions across the NuttX ecosystem:

1. **System Initialization (``netinit``)**: Automatically acquires an IP address at
   system startup when ``CONFIG_NETINIT_DHCPC`` is enabled.
2. **Interactive NSH Command**: Enables runtime dynamic configuration via the NuttShell
   ``ifconfig <interface> dhcp`` command.
3. **High-Level Network Helper (``netlib``)**: Powers ``netlib_obtain_ipv4addr()`` in
   ``apps/netutils/netlib/`` to configure network interfaces with minimal application
   boilerplate.
4. **Programmatic C API**: Offers both synchronous (blocking) and asynchronous
   (callback-driven) APIs for custom application-level interface management and lease
   lifecycles.

Key Features
============

- **RFC 2131 & RFC 1542 Compliance**: Standard four-step message handshake
  (``DISCOVER``, ``OFFER``, ``REQUEST``, ``ACK``) with optional ``RELEASE`` handling.
- **Interface Binding**: Binds client sockets directly to the target network device
  using ``SO_BINDTODEVICE``, ensuring reliable operation on multihomed devices.
- **Lease Timing & Lifetime Tracking**: Computes renewal timer :math:`T_1` (typically
  50% of lease time), rebinding timer :math:`T_2` (typically 87.5% of lease time), and
  expiration time for lease renewal scheduling.
- **DNS & NTP Server Discovery**: Parses option 6 (Domain Name Server) and option 42
  (Network Time Protocol Servers) into structured address arrays.
- **Graceful Address Release**: Implements ``dhcpc_release()`` with configurable
  transmission delays and retry mechanisms to inform the DHCP server when an address
  is surrendered.
- **Dual Execution Modes**: Supports both synchronous operation (``dhcpc_request()``
  blocking until negotiation completes or retries are exhausted) and asynchronous
  operation (``dhcpc_request_async()`` running in a dedicated worker thread with
  user callback notification).

Protocol Architecture & State Machine
=====================================

The DHCP client follows the finite state machine defined by :rfc:`2131`:

1. **INIT**: The client initializes on an unconfigured interface (IP ``0.0.0.0``) and
   broadcasts a ``DHCPDISCOVER`` message to ``255.255.255.255`` on UDP destination
   port 67.
2. **SELECTING**: The client listens for ``DHCPOFFER`` messages from one or more DHCP
   servers on UDP port 68. The first valid offer matching the transaction ID (``xid``)
   is selected.
3. **REQUESTING**: The client broadcasts a ``DHCPREQUEST`` message containing the
   server's identifier and requested IP address, confirming selection to all listening
   servers.
4. **BOUND**: Upon receipt of a valid ``DHCPACK`` message, the client enters the
   ``BOUND`` state. The assigned IP, netmask, gateway, lease time, and DNS servers are
   recorded.
5. **RENEWING / REBINDING**: At time :math:`T_1`, the client attempts unicast renewal
   with the original server. If unacknowledged by time :math:`T_2`, the client broadcasts
   to any available DHCP server.
6. **RELEASE**: When the interface is brought down or the application shuts down,
   ``dhcpc_release()`` transmits a unicast ``DHCPRELEASE`` message to the server,
   allowing the server to reclaim the address immediately.

Requirements & Dependencies
===========================

To use ``dhcpc``, ensure the following core networking and socket options are enabled
in your board configuration:

- Core Networking: ``CONFIG_NET=y``
- IPv4 Support: ``CONFIG_NET_IPv4=y``
- UDP Sockets: ``CONFIG_NET_UDP=y``
- UDP Broadcast Support: ``CONFIG_NET_BROADCAST=y`` (required for sending ``DHCPDISCOVER`` broadcast)
- Interface Socket Binding: ``CONFIG_NET_BINDTODEVICE=y`` (selected automatically by ``CONFIG_NETUTILS_DHCPC``)
- Client Library: ``CONFIG_NETUTILS_DHCPC=y``
- DNS Client (Optional, for DNS resolution): ``CONFIG_NETDB_DNSCLIENT=y``

Configuration Options
=====================

The DHCP client behavior can be fine-tuned via Kconfig under
``Application Configuration -> Network Utilities -> DHCP client``:

.. list-table::
   :widths: 35 15 50
   :header-rows: 1

   * - Kconfig Option
     - Default
     - Description
   * - ``CONFIG_NETUTILS_DHCPC``
     - ``n``
     - Enables compilation of the DHCP client library in ``apps/netutils/dhcpc``.
   * - ``CONFIG_NETUTILS_DHCPC_HOST_NAME``
     - ``"nuttx"``
     - Client hostname sent in DHCP option 12 to identify the device on the network.
   * - ``CONFIG_NETUTILS_DHCPC_RECV_TIMEOUT_MS``
     - ``3000``
     - Socket receive timeout in milliseconds when awaiting server responses.
   * - ``CONFIG_NETUTILS_DHCPC_RETRIES``
     - ``3``
     - Number of transmission attempts for ``DISCOVER`` and ``REQUEST`` before failing.
   * - ``CONFIG_NETUTILS_DHCPC_BOOTP_FLAGS``
     - ``0x0000``
     - BOOTP broadcast flags (:rfc:`1542`). Set to ``0x8000`` (broadcast) when IP
       forwarding is active or the device cannot accept unicast traffic before being
       configured.
   * - ``CONFIG_NETUTILS_DHCPC_RELEASE_RETRIES``
     - ``3``
     - Number of retry attempts when transmitting a ``DHCPRELEASE`` message.
   * - ``CONFIG_NETUTILS_DHCPC_RELEASE_TRANSMISSION_DELAY_MS``
     - ``10``
     - Delay in milliseconds between release retries and before socket closure,
       allowing the network stack to flush connectionless UDP packets.
   * - ``CONFIG_NETUTILS_DHCPC_RELEASE_ENSURE_TRANSMISSION``
     - ``y``
     - Inserts a transmission delay after sending ``DHCPRELEASE`` before closing the
       socket, ensuring the packet reaches the physical MAC driver.
   * - ``CONFIG_NETUTILS_DHCPC_RELEASE_CLEAR_IP``
     - ``n``
     - Clears the interface IP address, subnet mask, and default router immediately
       upon successful release transmission.
   * - ``CONFIG_NETUTILS_DHCPC_NTP_SERVERS``
     - ``1``
     - Maximum number of NTP server addresses to parse and record from DHCP option 42.
   * - ``CONFIG_NETDB_DNSSERVER_NAMESERVERS``
     - ``1``
     - Maximum number of DNS server addresses to parse and record from DHCP option 6.

NSH Usage
=========

When ``CONFIG_NETUTILS_DHCPC`` and ``CONFIG_NSH_NETLOCAL`` are enabled, the NuttShell
provides native runtime commands to request dynamic network addresses.

Configuring an Interface via DHCP
---------------------------------

To dynamically request an IP address on a specific network interface (e.g., ``eth0``
or ``wlan0``), issue the ``ifconfig`` command with the ``dhcp`` keyword:

.. code-block:: console

   nsh> ifconfig eth0 dhcp

NSH will invoke ``netlib_obtain_ipv4addr()``, which executes the DHCP exchange,
assigns the acquired IP address and subnet mask to ``eth0``, and updates the system
default gateway and DNS resolver entries.

Verifying the Acquired Configuration
------------------------------------

To display the active network parameters after DHCP completion:

.. code-block:: console

   nsh> ifconfig eth0
   eth0    HWaddr 00:e0:de:ad:be:ef at UP
           inet addr:192.168.1.150 DRaddr:192.168.1.1 Mask:255.255.255.0

Testing Network Reachability
----------------------------

Once the lease is acquired, verify reachability to local and external hosts:

.. code-block:: console

   nsh> ping 192.168.1.1
   nsh> ping 8.8.8.8

C API Reference
===============

The public C interface for ``dhcpc`` is declared in ``apps/include/netutils/dhcpc.h``.

Data Structures
---------------

.. code-block:: c

   struct dhcpc_state
   {
     struct in_addr serverid;                                   /* DHCP Server IPv4 address */
     struct in_addr ipaddr;                                     /* Client assigned IPv4 address */
     struct in_addr netmask;                                    /* Assigned subnet mask */
     struct in_addr dnsaddr[CONFIG_NETDB_DNSSERVER_NAMESERVERS]; /* Discovered DNS server(s) */
     uint8_t        num_dnsaddr;                                /* Count of valid DNS addresses */
     struct in_addr ntpaddr[CONFIG_NETUTILS_DHCPC_NTP_SERVERS];  /* Discovered NTP server(s) */
     uint8_t        num_ntpaddr;                                /* Count of valid NTP addresses */
     struct in_addr default_router;                             /* Default gateway router address */
     uint32_t       lease_time;                                 /* Total lease lifetime in seconds */
     uint32_t       renewal_time;                               /* T1: Seconds until RENEW state */
     uint32_t       rebinding_time;                             /* T2: Seconds until REBIND state */
   };

   typedef void (*dhcpc_callback_t)(FAR struct dhcpc_state *presult);

Core Functions
--------------

Initialization & Teardown
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   FAR void *dhcpc_open(FAR const char *interface,
                        FAR const void *mac_addr, int mac_len);
   void dhcpc_close(FAR void *handle);

- ``dhcpc_open()``: Allocates and initializes an internal DHCP client state context
  bound to the specified network interface name (e.g., ``"eth0"``).

  - ``interface``: Network device name (used with ``SO_BINDTODEVICE``).
  - ``mac_addr``: Pointer to the hardware (MAC) address buffer.
  - ``mac_len``: Length of the hardware address (typically ``IFHWADDRLEN``, 6 bytes).
  - *Returns*: An opaque context handle on success, or ``NULL`` if socket
    creation or memory allocation fails.

- ``dhcpc_close()``: Frees all allocated memory and closes active client sockets
  associated with the handle.

Synchronous Lease Request
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int dhcpc_request(FAR void *handle, FAR struct dhcpc_state *presult);

- Executes the blocking four-way DHCP handshake.
- ``handle``: Context handle returned by ``dhcpc_open()``.
- ``presult``: Pointer to a caller-allocated ``struct dhcpc_state`` structure that
  receives the negotiated network parameters.
- *Returns*: ``OK`` (0) on success, or ``ERROR`` (-1) on failure with ``errno`` set
  appropriately to indicate the cause of the failure:

  - ``ETIMEDOUT``: Retries exhausted without receiving a valid ``DHCPOFFER`` or ``DHCPACK``.
  - ``ECONNREFUSED``: The DHCP server rejected the request with a ``DHCPNAK``.
  - ``EINTR``: The request was interrupted or canceled via ``dhcpc_cancel()``.

Asynchronous Lease Request & Cancellation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int  dhcpc_request_async(FAR void *handle, dhcpc_callback_t callback);
   void dhcpc_cancel(FAR void *handle);

- ``dhcpc_request_async()``: Spawns a dedicated background pthread that executes the
  DHCP exchange non-blockingly. When the lease is obtained, the specified ``callback``
  is invoked with the populated ``struct dhcpc_state`` pointer.
- *Returns*: ``OK`` (0) on success, or ``ERROR`` (-1) on failure with ``errno`` set
  (e.g., ``EINVAL`` if parameters are invalid, or ``EALREADY`` if a DHCP thread is
  already running).
- ``dhcpc_cancel()``: Aborts a running asynchronous DHCP request thread.

Graceful Address Release
~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: c

   int dhcpc_release(FAR void *handle, FAR struct dhcpc_state *presult);

- Transmits a unicast ``DHCPRELEASE`` message to the server identified by
  ``presult->serverid`` for the assigned IP ``presult->ipaddr``.
- *Returns*: ``OK`` (0) on success, or ``ERROR`` (-1) on failure with ``errno`` set.

Application Examples
====================

Example 1: Synchronous IP Acquisition and Interface Configuration
-----------------------------------------------------------------

Below is a complete, production-grade example illustrating how an embedded application
queries the MAC address of an Ethernet adapter, executes a synchronous DHCP handshake,
and applies the negotiated parameters to the NuttX network stack:

.. code-block:: c

   #include <nuttx/config.h>
   #include <stdio.h>
   #include <string.h>
   #include <errno.h>
   #include <arpa/inet.h>
   #include <net/if.h>

   #include <netutils/netlib.h>
   #include <netutils/dhcpc.h>

   int configure_dhcp(FAR const char *ifname)
   {
     struct dhcpc_state state;
     uint8_t mac[IFHWADDRLEN];
     FAR void *handle;
     int ret;

     /* 1. Retrieve the hardware MAC address of the interface */

     ret = netlib_getmacaddr(ifname, mac);
     if (ret < 0)
       {
         fprintf(stderr, "Failed to get MAC address for %s: %d\n", ifname, ret);
         return ret;
       }

     /* 2. Open DHCP client session bound to the interface */

     handle = dhcpc_open(ifname, mac, sizeof(mac));
     if (handle == NULL)
       {
         fprintf(stderr, "Failed to initialize DHCP client handle\n");
         return -ENOMEM;
       }

     /* 3. Execute synchronous DHCP negotiation */

     printf("Sending DHCP request on %s...\n", ifname);
     ret = dhcpc_request(handle, &state);
     if (ret != OK)
       {
         int errcode = errno;
         fprintf(stderr, "DHCP request failed (errno %d: %s)\n",
                 errcode, strerror(errcode));
         dhcpc_close(handle);
         return -errcode;
       }

     /* 4. Apply negotiated parameters to the network stack */

     netlib_set_ipv4addr(ifname, &state.ipaddr);
     netlib_set_ipv4netmask(ifname, &state.netmask);
     netlib_set_dripv4addr(ifname, &state.default_router);

   #if defined(CONFIG_NETDB_DNSCLIENT)
     if (state.num_dnsaddr > 0)
       {
         netlib_set_ipv4dnsaddr(&state.dnsaddr[0]);
       }
   #endif

     printf("DHCP lease acquired successfully:\n");
     printf("  IP Address:     %s\n", inet_ntoa(state.ipaddr));
     printf("  Subnet Mask:    %s\n", inet_ntoa(state.netmask));
     printf("  Default Router: %s\n", inet_ntoa(state.default_router));
     printf("  Lease Lifetime: %lu seconds\n", (unsigned long)state.lease_time);

     /* 5. Clean up DHCP client session handle */

     dhcpc_close(handle);
     return OK;
   }

Example 2: Graceful Release on System Shutdown
----------------------------------------------

Before putting an embedded device into deep sleep or tearing down an active network
session, releasing the lease allows the DHCP server to reallocate the IP address
immediately:

.. code-block:: c

   int release_dhcp(FAR const char *ifname, FAR struct dhcpc_state *lease)
   {
     uint8_t mac[IFHWADDRLEN];
     FAR void *handle;
     int ret;

     if (netlib_getmacaddr(ifname, mac) < 0)
       {
         return -EINVAL;
       }

     handle = dhcpc_open(ifname, mac, sizeof(mac));
     if (handle == NULL)
       {
         return -ENOMEM;
       }

     printf("Releasing DHCP lease for %s...\n", inet_ntoa(lease->ipaddr));
     ret = dhcpc_release(handle, lease);
     if (ret != OK)
       {
         int errcode = errno;
         fprintf(stderr, "DHCP release failed (errno %d: %s)\n",
                 errcode, strerror(errcode));
         dhcpc_close(handle);
         return -errcode;
       }

     dhcpc_close(handle);
     return OK;
   }

Embedded Design & Tuning Guidelines
===================================

When deploying ``dhcpc`` in real-world embedded and IoT systems, consider the following
operational factors:

1. **Lease Renewal Strategy**: The ``dhcpc_request()`` function performs initial
   lease acquisition but does not maintain a permanent daemon thread. For long-running
   systems where lease durations are finite (e.g., cloud or office DHCP pools with
   1-hour or 24-hour leases), the application should maintain a software timer set
   to ``state.renewal_time`` (T1) to initiate renewal before expiration.
2. **Wireless Retransmission Tuning**: On lossy or congested Wi-Fi channels,
   increasing ``CONFIG_NETUTILS_DHCPC_RETRIES`` (e.g., from 3 to 5) and
   ``CONFIG_NETUTILS_DHCPC_RECV_TIMEOUT_MS`` (e.g., to 5000ms) improves connection
   reliability during initial radio association.
3. **Multihomed Network Binding**: Because ``dhcpc_open()`` sets
   ``SO_BINDTODEVICE`` on the underlying UDP socket, multiple interfaces (such as
   an Ethernet fallback and a primary Wi-Fi station) can execute independent DHCP
   sessions simultaneously without packet cross-talk.
