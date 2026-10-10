==================================
``netinit`` Network Initialization
==================================

The ``netinit`` utility is the standard network interface bring-up and configuration
subsystem in Apache NuttX (``apps/netutils/netinit``). It serves as the primary bridge
between lower-half network hardware drivers (Ethernet MAC/PHY, IEEE 802.11 Wi-Fi, 6LoWPAN,
SLIP, TUN, Loopback, SocketCAN) and upper-half network application layers (NuttX socket
layer, ``netlib``, and NuttShell).

``netinit`` coordinates the full boot-up sequence of network interfaces:

- **MAC Address Assignment**: Configures fixed software MACs, board unique ID (UID)
  locally administered addresses (LAA), or device calibration file MACs.
- **Link Layer Configuration**: Identifies and initializes the primary interface
  (e.g., ``eth0``, ``wlan0``, ``wpan0``).
- **IP Addressing**: Assigns static IPv4 addresses or coordinates dynamic configuration
  via the DHCP client (:rfc:`2131`), static IPv6 addresses or ICMPv6 Stateless Address
  Autoconfiguration (SLAAC, :rfc:`4862`), or runtime configuration via ``ipcfg`` files.
- **Wi-Fi Association**: Integrates with Wireless Application Programming Interface
  (WAPI) to automatically associate Wi-Fi interfaces with an Access Point (WPA/WPA2-PSK).
- **DNS Registration**: Registers default nameserver addresses with the resolver.
- **Auxiliary Network Services**: Optionally auto-launches background daemons such
  as the NTP client (``ntpclient``).
- **Autonomous Link Monitoring**: Runs an optional background event-driven monitor that
  detects PHY carrier changes, manages graceful link up/down transitions, and retries
  bring-up after disconnection.

Architecture and Operational Modes
==================================

``netinit`` supports three primary architectural execution models to accommodate different
embedded memory, real-time, and hardware constraints.

.. code-block:: text

   +-------------------------------------------------------------------------+
   |                       System Boot (nsh_init / nxinit)                  |
   +-------------------------------------------------------------------------+
                                        |
                                        v
                            netinit_bringup()
                                        |
                 +----------------------+----------------------+
                 |                                             |
                 v (CONFIG_NETINIT_THREAD=n)                   v (CONFIG_NETINIT_THREAD=y)
        Sequential Foreground                         Spawn "netinit" Thread
                 |                                    (Priority 80 / Detached)
                 +----------------------+----------------------+
                                        |
                                        v
   +-------------------------------------------------------------------------+
   |                    Phase 1: Local Device Initialization                 |
   |  - Select Primary Device (wlan0 / eth0 / wpan0 / bnep0 / sl0 / tun0)    |
   |  - Provision MAC Address (UIDMAC / Device Info / Fixed Software MAC)    |
   |  - Provision Local IP/Mask (Static Kconfig / fsutils ipcfg)             |
   +-------------------------------------------------------------------------+
                                        |
                                        +-----> (If CONFIG_NETINIT_NETLOCAL=y: Exit)
                                        |
                                        v
   +-------------------------------------------------------------------------+
   |                    Phase 2: Network Interface Bring-Up                  |
   |  - Bring Interface UP (netlib_ifup)                                     |
   |  - Associate Wi-Fi AP via WAPI (wpa_driver_wext_associate)              |
   |  - Acquire Dynamic IP via DHCPC or ICMPv6 SLAAC                         |
   |  - Register Default DNS Nameserver                                      |
   |  - Start NTP Client Daemon (ntpc_start)                                 |
   +-------------------------------------------------------------------------+
                                        |
                 +----------------------+----------------------+
                 |                                             |
                 v (CONFIG_NETINIT_MONITOR=n)                  v (CONFIG_NETINIT_MONITOR=y)
            Exit Thread /                               Persistent MII Monitor
          Release Resources                              (SIOCMIINOTIFY Event Loop)

Sequential Initialization Mode
------------------------------

When ``CONFIG_NETINIT_THREAD`` is disabled (default), ``netinit_bringup()`` executes
synchronously on the calling task's thread (for example, during NSH startup in ``nsh_init()``
or system startup in ``nxinit``).

- **Advantages**: Consumes zero additional thread stacks or task management overhead;
  ideal for microcontrollers with very limited RAM (e.g., <= 64 KB).
- **Caveats**: If the physical Ethernet cable is unplugged, or if the PHY auto-negotiation
  or DHCP acquisition requires several seconds, system boot will block until timeouts
  expire.

Threaded Asynchronous Mode
--------------------------

When ``CONFIG_NETINIT_THREAD=y`` is selected, ``netinit_bringup()`` spawns a dedicated
background POSIX thread (named ``"netinit"``) with configurable stack size
(``CONFIG_NETINIT_THREAD_STACKSIZE``) and priority (``CONFIG_NETINIT_THREAD_PRIORITY``,
typically set to low priority, such as 80 or 100).

- **Advantages**: System boot proceeds immediately. Applications, the NuttShell console,
  and local real-time tasks start instantly without blocking on network auto-negotiation.
- **Lifecycle**: The thread configures the network interface, performs DHCP or Wi-Fi
  association, and once the network is operational, terminates cleanly to reclaim its
  allocated resources (unless the Link State Monitor is enabled).

Local vs. Network Bringup Split
-------------------------------

When ``CONFIG_NETINIT_NETLOCAL=y`` is enabled, ``netinit`` performs only local interface
provisioning:

1. Software MAC address assignment (if hardware requires it).
2. Local IP address and subnet assignment on the network device.

It explicitly suppresses:

- Bringing the interface state UP via ``netlib_ifup()``.
- Wi-Fi Access Point association via WAPI.
- Dynamic IP negotiation via the DHCP client.
- Starting the background NTP client daemon.

This split mode is particularly useful in multi-stage boot environments where wireless
scanning, captive portal redirection, or user credential entry must occur before bringing
the network interface online.

Link State Monitor Daemon
-------------------------

When ``CONFIG_NETINIT_MONITOR=y`` is selected (requiring ``CONFIG_NETINIT_THREAD=y``,
``CONFIG_ARCH_PHY_INTERRUPT=y``, ``CONFIG_NETDEV_PHY_IOCTL=y``, and ``CONFIG_NET_UDP=y``),
the ``netinit`` thread persists permanently as an event-driven daemon:

1. **Signal Attachment**: Registers a signal handler for ``CONFIG_NETINIT_SIGNO``
   (default real-time signal 32) using ``sigaction()``.
2. **PHY Event Registration**: Issues an ``ioctl(sd, SIOCMIINOTIFY, ...)`` command to the
   network driver, requesting asynchronous signal delivery upon MII PHY link status
   changes.
3. **Carrier Detection**: Reads PHY status registers (``SIOCGMIIPHY`` and ``SIOCGMIIREG``
   at register ``MII_MSR``) to evaluate ``MII_MSR_LINKSTATUS``.
4. **Transition to Up**: When the link transitions from down to up, ``netinit`` marks
   the device operational (``IFF_UP``), re-executes ICMPv6 auto-configuration if enabled,
   and sleeps until the next interrupt.
5. **Transition to Down**: When cable disconnection is detected, ``netinit`` clears
   the ``IFF_UP`` flag and periodically attempts reconnection every
   ``CONFIG_NETINIT_RETRYMSEC`` milliseconds (default 2000 ms).

Device Selection Hierarchy
==========================

``netinit`` identifies the primary network interface name (``NET_DEVNAME``) at compile
time based on enabled link-layer drivers, adhering to the following selection precedence:

.. list-table::
   :widths: 25 20 55
   :header-rows: 1

   * - Interface Name
     - Configuration Guard
     - Description
   * - ``wlan0``
     - ``CONFIG_DRIVERS_IEEE80211``
     - IEEE 802.11 wireless network adapter (Wi-Fi).
   * - ``eth0``
     - ``CONFIG_NET_ETHERNET``
     - Standard 10/100/1000 Mbps Ethernet MAC/PHY.
   * - ``wpan0``
     - ``CONFIG_NET_6LOWPAN`` or ``CONFIG_NET_IEEE802154``
     - Low-power wireless personal area network (6LoWPAN / Zigbee).
   * - ``bnep0``
     - ``CONFIG_NET_BLUETOOTH``
     - Bluetooth Network Encapsulation Protocol interface.
   * - ``sl0``
     - ``CONFIG_NET_SLIP``
     - Serial Line Internet Protocol (SLIP) point-to-point interface.
   * - ``tun0``
     - ``CONFIG_NET_TUN``
     - Virtual TUN/TAP network tunnel adapter.
   * - ``lo``
     - ``CONFIG_NET_LOOPBACK``
     - Local loopback interface (``127.0.0.1``).
   * - ``can0``
     - ``CONFIG_NET_CAN``
     - SocketCAN controller interface.

MAC Address Assignment Strategies
=================================

Embedded network controllers often lack factory-programmed MAC addresses in non-volatile
hardware registers. When ``CONFIG_NETINIT_NOMAC=y`` is set, ``netinit`` provisions a
valid MAC address before bringing the interface up using one of three strategies:

Board Unique ID (``CONFIG_NETINIT_UIDMAC``)
-------------------------------------------

Queries the microcontroller's factory-flashed unique silicon ID (e.g., STM32 96-bit UID,
ESP32 eFuse MAC) by invoking:

.. code-block:: c

   boardctl(BOARDIOC_UNIQUEID, (uintptr_t)&uid);

The first byte is adjusted to set the IEEE Locally Administered Address (LAA) bit and
clear the multicast bit:

.. code-block:: c

   uid[0] = (uid[0] & 0xf0) | 0x02; /* Locally Administered MAC */
   netlib_setmacaddr(NET_DEVNAME, uid);

This allows boards booted from the same firmware binary to dynamically generate
a unique, UID-derived MAC address on the local network without hardcoding. Note that
because 64-bit or 96-bit MCU hardware UIDs are truncated to 48 bits (6 bytes for Ethernet)
or 64 bits (for 6LoWPAN), collision avoidance depends on the uniqueness of the lower UID
bytes exposed by the hardware platform.

Device Info File (``CONFIG_NETINIT_MACADDR``)
---------------------------------------------

Loads a calibrated, production-programmed MAC address from persistent non-volatile
storage (e.g., ``/etc/device.info`` or EEPROM) via:

.. code-block:: c

   boardctl(BOARDIOC_MACADDR, (uintptr_t)&req);

Fixed Software MAC (``CONFIG_NETINIT_SWMAC``)
---------------------------------------------

Assigns a static MAC address defined directly in Kconfig:

- ``CONFIG_NETINIT_MACADDR_1``: Lower 4 bytes (e.g., ``0xdeadbeef``).
- ``CONFIG_NETINIT_MACADDR_2``: Upper 2 bytes (e.g., ``0x00e0`` for Ethernet).

IP Address Configuration
========================

IPv4 Addressing
---------------

Dynamic DHCP Client (``CONFIG_NETINIT_DHCPC``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When ``CONFIG_NETINIT_DHCPC=y`` is selected (requires ``CONFIG_NETUTILS_DHCPC=y``),
``netinit`` initializes the device address to ``0.0.0.0``, brings the link up, and
calls:

.. code-block:: c

   netlib_obtain_ipv4addr(NET_DEVNAME);

The DHCP client broadcasts a ``DHCPDISCOVER`` packet, negotiates lease terms with the local
DHCP server, and automatically applies the assigned IP address, network mask, default
gateway router, and DNS nameservers.

Static IPv4 Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

When DHCP is disabled, static addresses are supplied via Kconfig in 32-bit host byte order:

- ``CONFIG_NETINIT_IPADDR``: Target host IP (e.g., ``0x0a000002`` for ``10.0.0.2``).
- ``CONFIG_NETINIT_DRIPADDR``: Default router IP (e.g., ``0x0a000001`` for ``10.0.0.1``).
- ``CONFIG_NETINIT_NETMASK``: Subnet mask (e.g., ``0xffffff00`` for ``255.255.255.0``).

Runtime File-Based Configuration (``CONFIG_FSUTILS_IPCFG``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If ``CONFIG_FSUTILS_IPCFG=y`` is enabled, ``netinit`` attempts to read network settings
at runtime from a persistent IP configuration file using ``ipcfg_read()``.

**Storage Path (CONFIG_IPCFG_PATH)**
  The location of the configuration file is defined by ``CONFIG_IPCFG_PATH`` (defaulting
  to ``/etc/sysconfig/network-scripts``). In standard filesystem mode, ``netinit`` resolves
  the per-interface configuration file as ``<CONFIG_IPCFG_PATH>/ipcfg-<netdev>``
  (for example, ``/etc/sysconfig/network-scripts/ipcfg-eth0``).

  Alternatively, on deeply embedded single-interface targets lacking a mounted file system,
  ``CONFIG_IPCFG_CHARDEV=y`` allows ``CONFIG_IPCFG_PATH`` to specify the direct path to a
  character driver (such as an EEPROM or FLASH memory node, e.g., ``/dev/eeprom``).

**File Formats (ASCII vs. Binary)**
  The configuration format is determined by ``CONFIG_IPCFG_BINARY``:

  - **ASCII Format (Default, ``CONFIG_IPCFG_BINARY=n``)**:
    Stored as a human-readable text file composed of standard ``<variable>=<value>`` pairs
    (e.g., ``DEVICE=eth0``, ``BOOTPROTO=static``, ``IPADDR=192.168.1.50``,
    ``NETMASK=255.255.255.0``, ``ROUTER=192.168.1.1``, ``DNS=8.8.8.8``).

  - **Binary Format (``CONFIG_IPCFG_BINARY=y``)**:
    Stored as a compact, fixed-size binary structure (``struct ipcfg_s``). This eliminates
    string parsing overhead and is specifically tailored for constrained non-volatile media
    such as EEPROM or raw flash blocks.

**Operational Precedence**
  If the configuration file specifies ``IPv4PROTO_DHCP`` (or ``BOOTPROTO=dhcp``), dynamic
  DHCP client negotiation is performed. If static parameters are defined, the file's
  static IP, gateway, and netmask override the compile-time Kconfig defaults. If the backing
  filesystem mounts asynchronously after task boot, ``CONFIG_NETINIT_RETRY_MOUNTPATH``
  specifies the number of retry iterations ``netinit`` performs before falling back.

IPv6 Addressing
---------------

Stateless Address Autoconfiguration (SLAAC)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When ``CONFIG_NET_ICMPv6_AUTOCONF=y`` is set, ``netinit`` invokes:

.. code-block:: c

   netlib_icmpv6_autoconfiguration(NET_DEVNAME);

The interface transmits an ICMPv6 Router Solicitation and processes Router Advertisement
prefixes to construct its global IPv6 address per :rfc:`4862`.

Static IPv6 Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^

If SLAAC is not used, static IPv6 addresses are defined as eight 16-bit hex values in host
order:

- Host IP: ``CONFIG_NETINIT_IPv6ADDR_1`` through ``CONFIG_NETINIT_IPv6ADDR_8`` (defaults
  to ``fc00::2``).
- Gateway: ``CONFIG_NETINIT_DRIPv6ADDR_1`` through ``CONFIG_NETINIT_DRIPv6ADDR_8``
  (defaults to ``fc00::1``).
- Netmask: ``CONFIG_NETINIT_IPv6NETMASK_1`` through ``CONFIG_NETINIT_IPv6NETMASK_8``.
  In ``Kconfig``, words 1 through 7 default to ``0xffff`` while word 8 defaults to
  ``0x0000``, producing an effective default netmask of
  ``ffff:ffff:ffff:ffff:ffff:ffff:ffff:0`` (``/112``). If a standard prefix such as
  ``/64`` (``ffff:ffff:ffff:ffff::``) is required, the individual netmask words should
  be set explicitly in your board configuration.

DNS Nameserver Configuration
----------------------------

When ``CONFIG_NETINIT_DNS=y`` is enabled (depends on ``CONFIG_NETDB_DNSCLIENT=y``),
``netinit`` configures the default nameserver:

- Defaults to the default gateway address (``CONFIG_NETINIT_DRIPADDR``) or
  ``CONFIG_NETINIT_DNSIPADDR``.
- The address is committed using ``netlib_set_ipv4dnsaddr()``.

Wi-Fi Association (WAPI Integration)
====================================

For IEEE 802.11 Wi-Fi adapters (``CONFIG_DRIVERS_IEEE80211=y`` and ``CONFIG_WIRELESS_WAPI=y``),
``netinit`` automatically coordinates wireless association with an Access Point via
``netinit_associate("wlan0")``.

Parameters configured via Kconfig or runtime WAPI files:

- **Station Mode**: Infrastructure (``CONFIG_NETINIT_WAPI_STAMODE_INFRA``), Ad-hoc,
  Repeater, Monitor, or Mesh.
- **Authentication**: WPA2-Personal (``CONFIG_NETINIT_WAPI_AUTHWPA_WPA2``), WPA-Personal,
  or Disabled/Open.
- **Cipher**: CCMP/AES (``CONFIG_NETINIT_WAPI_CIPHERMODE_CCMP``), TKIP, or WEP.
- **Algorithm**: ``WPA_ALG_CCMP`` (``CONFIG_NETINIT_WAPI_ALG_CCMP``).
- **Credentials**: ``CONFIG_NETINIT_WAPI_SSID`` and ``CONFIG_NETINIT_WAPI_PASSPHRASE``.

NTP Client Synchronization
==========================

If ``CONFIG_NETUTILS_NTPCLIENT=y`` is selected, ``netinit`` automatically starts the
Network Time Protocol daemon as soon as interface bring-up succeeds:

.. code-block:: c

   ntpc_start();

This ensures that system clocks on embedded targets without hardware battery-backed Real-Time
Clocks (RTC) synchronize immediately upon network establishment.

Configuration Reference
=======================

The following table summarizes key configuration options in ``apps/netutils/netinit/Kconfig``:

.. list-table::
   :widths: 35 15 15 35
   :header-rows: 1

   * - Symbol
     - Type
     - Default
     - Description
   * - ``CONFIG_NETUTILS_NETINIT``
     - boolean
     - ``n``
     - Master enable switch for the network initialization subsystem.
   * - ``CONFIG_NETINIT_NETLOCAL``
     - boolean
     - ``n``
     - Restricts initialization to local MAC and IP setup; does not bring link UP.
   * - ``CONFIG_NETINIT_THREAD``
     - boolean
     - ``n``
     - Executes network bring-up asynchronously in a dedicated background pthread.
   * - ``CONFIG_NETINIT_THREAD_PRIORITY``
     - integer
     - ``80``
     - Priority of the asynchronous initialization thread.
   * - ``CONFIG_NETINIT_THREAD_STACKSIZE``
     - integer
     - ``DEFAULT_TASK_STACKSIZE``
     - Stack size in bytes for the ``netinit`` thread.
   * - ``CONFIG_NETINIT_MONITOR``
     - boolean
     - ``n``
     - Enables autonomous MII PHY carrier link status monitoring daemon.
   * - ``CONFIG_NETINIT_SIGNO``
     - integer
     - ``32``
     - Real-time signal number used for asynchronous PHY event notifications.
   * - ``CONFIG_NETINIT_RETRYMSEC``
     - integer
     - ``2000``
     - Retry period in milliseconds when attempting to reconnect a dropped link.
   * - ``CONFIG_NETINIT_RETRY_MOUNTPATH``
     - integer
     - ``0``
     - Number of retries waiting for filesystem mount before reading ``ipcfg``.
   * - ``CONFIG_IPCFG_PATH``
     - string
     - ``"/etc/sysconfig/network-scripts"``
     - Directory path or character device node for IP configuration storage (via ``fsutils/ipcfg``).
   * - ``CONFIG_IPCFG_BINARY``
     - boolean
     - ``n``
     - Selects compact binary format over human-readable ASCII text for ``ipcfg``.
   * - ``CONFIG_NETINIT_DEBUG``
     - boolean
     - ``n``
     - Enables verbose unit-level debug traces without enabling global ``DEBUG_NET``.
   * - ``CONFIG_NETINIT_DHCPC``
     - boolean
     - ``n``
     - Uses DHCP client (``netutils/dhcpc``) to dynamically acquire an IPv4 address.
   * - ``CONFIG_NETINIT_IPADDR``
     - hex
     - ``0x0a000002``
     - Static host IPv4 address in host order (e.g., ``10.0.0.2``).
   * - ``CONFIG_NETINIT_DRIPADDR``
     - hex
     - ``0x0a000001``
     - Static default router (gateway) IPv4 address (e.g., ``10.0.0.1``).
   * - ``CONFIG_NETINIT_NETMASK``
     - hex
     - ``0xffffff00``
     - Static IPv4 subnet mask (e.g., ``255.255.255.0``).
   * - ``CONFIG_NETINIT_DNS``
     - boolean
     - ``n``
     - Enables default DNS nameserver address registration.
   * - ``CONFIG_NETINIT_DNSIPADDR``
     - hex
     - ``0xa0000001``
     - DNS nameserver IPv4 address in host order.
   * - ``CONFIG_NETINIT_NOMAC``
     - boolean
     - ``n``
     - Enables software MAC assignment for hardware lacking factory MAC storage.
   * - ``CONFIG_NETINIT_UIDMAC``
     - boolean
     - ``n``
     - Generates an IEEE LAA MAC address derived from the board's unique silicon ID.
   * - ``CONFIG_NETINIT_SWMAC``
     - boolean
     - ``n``
     - Assigns a fixed software MAC address configured in Kconfig.
   * - ``CONFIG_NETINIT_MACADDR``
     - boolean
     - ``n``
     - Queries MAC address from device calibration file via ``BOARDIOC_MACADDR``.
   * - ``CONFIG_NETINIT_WAPI_SSID``
     - string
     - ``""``
     - Wi-Fi Access Point Service Set Identifier (SSID) for auto-association.
   * - ``CONFIG_NETINIT_WAPI_PASSPHRASE``
     - string
     - ``""``
     - Wi-Fi WPA/WPA2 pre-shared key (passphrase).

Application Programming Interface (C API)
=========================================

Header File
-----------

Applications and board initialization files access ``netinit`` routines by including:

.. code-block:: c

   #include "netutils/netinit.h"

Functions
---------

.. c:function:: int netinit_bringup(void)

   Initializes and configures the primary network interface according to the selected
   NuttX configuration.

   If ``CONFIG_NETINIT_THREAD`` is disabled, execution is synchronous: MAC addresses are
   assigned, IP addresses configured, the interface brought up, and any dynamic services
   (DHCP, WAPI, NTP) started before the function returns.

   If ``CONFIG_NETINIT_THREAD=y`` is enabled, this function initializes thread attributes,
   creates the detached background pthread named ``"netinit"``, and returns ``OK``
   immediately.

   :return: ``OK`` (0) on success; a negated ``errno`` value on failure (e.g., if thread
            creation fails).

.. c:function:: int netinit_associate(FAR const char *ifname)

   Associates an IEEE 802.11 wireless network interface with a Wi-Fi Access Point using
   WAPI and WEXT driver ioctls.

   Loads configuration via ``wapi_load_config()`` if available, or falls back to
   ``CONFIG_NETINIT_WAPI_*`` compile-time parameters.

   :param ifname: Name of the wireless interface (e.g., ``"wlan0"``).
   :return: ``OK`` (0) on successful association; a negated ``errno`` on failure.

Usage Examples
==============

Scenario 1: Standard Ethernet Bring-Up with DHCP
------------------------------------------------

To configure a standard board (such as an STM32, SAMv7, or ESP32 Ethernet design) to
acquire an IP address via DHCP in the background without blocking console startup, enable
the following in your board's ``defconfig``:

.. code-block:: kconfig

   CONFIG_NET=y
   CONFIG_NET_ETHERNET=y
   CONFIG_NETUTILS_NETLIB=y
   CONFIG_NETUTILS_NETINIT=y
   CONFIG_NETUTILS_DHCPC=y
   CONFIG_NETINIT_DHCPC=y
   CONFIG_NETINIT_THREAD=y
   CONFIG_NETINIT_THREAD_PRIORITY=80
   CONFIG_NETINIT_THREAD_STACKSIZE=2048

In your board initialization or custom application:

.. code-block:: c

   #include <nuttx/config.h>
   #include <stdio.h>
   #include "netutils/netinit.h"

   int main(int argc, char *argv[])
   {
     int ret;

     printf("Bringing up network interface...\n");
     ret = netinit_bringup();
     if (ret < 0)
       {
         fprintf(stderr, "ERROR: netinit_bringup failed: %d\n", ret);
         return ret;
       }

     printf("Network bringup initiated in background thread.\n");
     return 0;
   }

Scenario 2: Industrial Device with Board Unique ID MAC & Static IP
------------------------------------------------------------------

For an industrial microcontroller lacking an onboard EEPROM MAC, derive the MAC address
from silicon UID and assign static IP parameters:

.. code-block:: kconfig

   CONFIG_NET=y
   CONFIG_NET_ETHERNET=y
   CONFIG_BOARDCTL_UNIQUEID=y
   CONFIG_BOARDCTL_UNIQUEID_SIZE=12
   CONFIG_NETUTILS_NETINIT=y
   CONFIG_NETINIT_NOMAC=y
   CONFIG_NETINIT_UIDMAC=y
   CONFIG_NETINIT_IPADDR=0xc0a80132      # 192.168.1.50
   CONFIG_NETINIT_DRIPADDR=0xc0a80101    # 192.168.1.1
   CONFIG_NETINIT_NETMASK=0xffffff00     # 255.255.255.0
   CONFIG_NETINIT_DNS=y
   CONFIG_NETINIT_DNSIPADDR=0x08080808   # 8.8.8.8

Scenario 3: Wi-Fi IoT Device with WPA2-PSK Auto-Association
-----------------------------------------------------------

For an embedded Wi-Fi station connecting automatically to a local router on boot:

.. code-block:: kconfig

   CONFIG_NET=y
   CONFIG_DRIVERS_IEEE80211=y
   CONFIG_WIRELESS_WAPI=y
   CONFIG_NETUTILS_NETINIT=y
   CONFIG_NETINIT_THREAD=y
   CONFIG_NETINIT_DHCPC=y
   CONFIG_NETINIT_WAPI_STAMODE_INFRA=y
   CONFIG_NETINIT_WAPI_AUTHWPA_WPA2=y
   CONFIG_NETINIT_WAPI_CIPHERMODE_CCMP=y
   CONFIG_NETINIT_WAPI_ALG_CCMP=y
   CONFIG_NETINIT_WAPI_SSID="MyHomeNetwork"
   CONFIG_NETINIT_WAPI_PASSPHRASE="SecretPassword123"

Troubleshooting and Diagnostics
===============================

Enabling Dedicated Debugging
----------------------------

Global network debugging (``CONFIG_DEBUG_NET`` with ``CONFIG_DEBUG_INFO``) generates high
volumes of packet-level log output that can flood serial consoles and distort timing.

To isolate debug output specifically to ``netinit`` without global log spam, enable:

.. code-block:: kconfig

   CONFIG_DEBUG_FEATURES=y
   CONFIG_NETINIT_DEBUG=y

This forces verbose ``ninfo()`` and ``nerr()`` diagnostics strictly within ``netinit.c``.

Common Issues
-------------

1. **System Freezes During Boot**:

   - *Cause*: ``CONFIG_NETINIT_THREAD`` is disabled, and Ethernet cable is unplugged or
     DHCP server is unreachable.
   - *Solution*: Enable ``CONFIG_NETINIT_THREAD=y`` so network negotiation occurs
     concurrently without blocking system startup.

2. **Duplicate MAC Address Conflicts**:

   - *Cause*: Multiple devices deployed with default fixed software MAC (``CONFIG_NETINIT_SWMAC``).
   - *Solution*: Switch to ``CONFIG_NETINIT_UIDMAC=y`` so each device generates a UID-derived
     IEEE LAA address from MCU silicon unique ID registers rather than sharing a static MAC.

3. **Link State Monitor Never Detects Reconnection**:

   - *Cause*: ``CONFIG_ARCH_PHY_INTERRUPT`` is missing, or board Ethernet driver does not
     implement MII ioctls (``SIOCMIINOTIFY``, ``SIOCGMIIPHY``, ``SIOCGMIIREG``).
   - *Solution*: Verify board PHY interrupt wiring and verify driver support in the board's
     Ethernet architecture layer.

4. **Wi-Fi Fails to Associate**:

   - *Cause*: Mismatched cipher/algorithm settings (e.g., Access Point requires WPA3 or
     TKIP while client is configured strictly for WPA2-CCMP), or incorrect SSID/passphrase.
   - *Solution*: Verify AP security policy and confirm SSID and passphrase via NSH ``wapi``
     command line utilities.
