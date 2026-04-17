============================
``ntpc`` NTP Daemon Commands
============================

This example demonstrates how to use the NTP client to synchronize system time
and retrieve the current time and date. It uses the Network Time Protocol (NTP)
to provide accurate time synchronization.

Description
-----------

The ntpc example:

- Uses the NTP client library to synchronize system time

- Connects to NTP servers (default: pool.ntp.org)

- Supports NTP server configuration learned from DHCP option 42

- Starts the NTP client in the background for continuous synchronization

- Provides commands to check status and stop the NTP client

- Allows management of the NTP client through command line options

Note: This example assumes that network connectivity is already established.

The NTP (Network Time Protocol) is a sophisticated protocol that provides
high-precision time synchronization and is the standard for network time services.

Configuration
-------------

This example requires the following NuttX configuration options:

- CONFIG_NET: Enable networking support
- CONFIG_NET_UDP: Enable UDP support
- CONFIG_NETUTILS_NTPCLIENT: Enable NTP client support
- CONFIG_SYSTEM_NTPC: Enable this example

Additional configuration options:

- CONFIG_NETUTILS_NTPCLIENT_SERVER: NTP server hostname (default: "pool.ntp.org")
- CONFIG_NETUTILS_DHCPC: Enable DHCP client support when NTP servers should be
  learned dynamically from DHCP option 42

Usage
-----

1. Configure your NuttX build with networking support
2. Ensure network connectivity is established (e.g., via NSH network commands)
   If DHCP provides NTP servers through option 42, ``ntpc`` can use that
   server list automatically.
3. Build and flash the image to your target board
4. Run the commands:
   - ``ntpcstart``, ``ntpcstop``, ``ntpcstatus``

**Available Commands:**

- ``ntpcstart`` - Start NTP client in background

- ``ntpcstop`` - Stop the NTP client

- ``ntpcstatus`` - Display NTP client status information

Expected Output
---------------

**Start NTP client (ntpcstart):**
::

   Starting NTP client...
   Using NTP servers: pool.ntp.org
   NTP client started successfully (task ID: 123)
   NTP client is now running in the background

**Stop NTP client (ntpcstop):**
::

   Stopping NTP client...
   Stopped the NTP daemon

**Show NTP status (ntpcstatus):**
::

    The number of last samples: 3
    [0] srv <ip> offset -0.033502142 delay 0.249973549
    [1] srv <ip> offset -0.020698070 delay 0.029928000
    [2] srv <ip> offset -0.015448935 delay 0.019815119

**Verify using date command:**

Given network connectivity is available, executing `date` should
give the proper time and date.
::

    nsh> ntpcstart
    Starting NTP client...
    Using NTP servers: 0.pool.ntp.org;1.pool.ntp.org;2.pool.ntp.org
    NTP client started successfully (task ID: 10)
    NTP client is now running in the background
    nsh> date
    Fri, Sep 05 18:49:37 2025

DHCP-provided NTP servers
-------------------------

When ``CONFIG_NETUTILS_DHCPC`` is enabled, the DHCP client can pass NTP
server IPv4 addresses learned from DHCP option 42 to ``ntpc``.

This allows ``ntpc`` to run without a fixed server hostname in the
configuration and to follow NTP server updates delivered by DHCP.

One way to test DHCP-delivered NTP servers is with ``dnsmasq``:

::

   dnsmasq --no-daemon --log-dhcp --log-queries \
     --interface=tap0 --bind-interfaces \
     --dhcp-authoritative \
     --dhcp-range=192.168.50.20,192.168.50.50,255.255.255.0 \
     --dhcp-option=option:router,192.168.50.1 \
     --dhcp-option=option:dns-server,1.1.1.1 \
     --dhcp-option=option:ntp-server,162.159.200.123

Notes
-----

- This example requires internet connectivity
- Network must be configured and connected before running this example
- NTP servers must be accessible (default: pool.ntp.org)
- UDP port 123 (NTP) must not be blocked by firewalls
- The example includes error handling for network failures
- NTP provides more accurate time synchronization than simple time protocols
