========================
``ntpclient`` NTP client
========================

The NTP (Network Time Protocol) client is a network utility that synchronizes
the system clock with time servers over the Internet. This implementation
provides a minimal but functional NTP client for NuttX.

What is NTP?
============

The Network Time Protocol (NTP) is a networking protocol designed to
synchronize clocks of computer systems over packet-switched, variable-latency
data networks. NTP is one of the oldest Internet protocols still in use,
originally designed by David L. Mills of the University of Delaware.

Key features of NTP:

- **High Precision**: NTP can achieve sub-millisecond accuracy on local area
  networks and typically 10-100 millisecond accuracy over the Internet
- **Robust Algorithm**: Uses sophisticated algorithms to filter out network
  jitter and select the best time sources
- **Hierarchical Structure**: Uses a stratum system where stratum 0 devices
  are atomic clocks, stratum 1 servers sync with stratum 0, and so on
- **Fault Tolerance**: Can handle multiple time sources and automatically
  switch between them

NTP Protocol Overview
=====================

NTP uses UDP port 123 and follows a client-server model. The protocol
exchanges timestamps to calculate:

- **Offset**: The difference between the client's clock and the server's clock
- **Delay**: The round-trip network delay
- **Dispersion**: The maximum error due to clock frequency tolerance

The NTP packet format (version 3) includes:

- **Leap Indicator**: Warns of an impending leap second
- **Version Number**: NTP version (3 in this implementation)
- **Mode**: Client, server, broadcast, etc.
- **Stratum**: Clock level (0-15)
- **Poll Interval**: Maximum interval between successive messages
- **Precision**: Clock precision
- **Root Delay/Dispersion**: Total delay and dispersion to the reference clock
- **Reference Identifier**: Identifies the reference source
- **Reference Timestamp**: Time when the system clock was last set
- **Originate Timestamp**: Time when the request departed the client
- **Receive Timestamp**: Time when the request arrived at the server
- **Transmit Timestamp**: Time when the reply departed the server

Implementation Details
======================

The NuttX NTP client implementation consists of several key components:

Source Code Structure
---------------------

**ntpclient.c** - Main implementation file containing:

- **Daemon Management**: Functions to start, stop, and manage the NTP daemon
- **Time Synchronization**: Core algorithms for calculating clock offset and delay
- **Network Communication**: UDP socket handling and NTP packet exchange
- **Sample Collection**: Gathering multiple time samples for statistical filtering

**ntpv3.h** - NTP version 3 packet format definitions:

- **ntp_datagram_s**: Complete NTP packet structure
- **ntp_timestamp_s**: 64-bit NTP timestamp format
- **Protocol Constants**: NTP version, modes, and stratum definitions

Key Functions
-------------

- **ntpc_start_with_list()**: Starts the NTP daemon with a list of servers
- **ntpc_start()**: Starts the NTP daemon with default configuration
- **ntpc_stop()**: Stops the running NTP daemon
- **ntpc_status()**: Retrieves current synchronization status and samples
- **ntpc_daemon()**: Main daemon loop that:

  - Connects to configured NTP servers
  - Sends NTP requests and processes responses
  - Calculates clock offset and delay
  - Applies time corrections to the system clock
  - Continues polling at configured intervals

- **ntpc_get_ntp_sample()**: Performs a single NTP transaction:

  - Creates UDP socket to NTP server
  - Sends NTP request packet with current timestamp
  - Receives and validates NTP response
  - Calculates offset and delay using NTP algorithms

- **ntpc_calculate_offset()**: Implements the NTP clock filter algorithm

  - Uses four timestamps, calculates offset and delay
  - Applies statistical filtering to reduce jitter

- **ntpc_settime()**: Applies time correction to system clock:

  - Uses calculated offset to adjust system time
  - Handles both positive and negative time adjustments
  - Maintains monotonic clock consistency

Configuration Options
=====================

The NTP client can be configured through Kconfig options:

- **CONFIG_NETUTILS_NTPCLIENT_SERVER**: List of NTP server hostnames
- **CONFIG_NETUTILS_NTPCLIENT_PORTNO**: NTP server port (default: 123)
- **CONFIG_NETUTILS_NTPCLIENT_STACKSIZE**: Daemon task stack size
- **CONFIG_NETUTILS_NTPCLIENT_SERVERPRIO**: Daemon task priority
- **CONFIG_NETUTILS_NTPCLIENT_STAY_ON**: Keep polling continuously
- **CONFIG_NETUTILS_NTPCLIENT_POLLDELAYSEC**: Polling interval in seconds
- **CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES**: Number of samples for filtering
- **CONFIG_NETUTILS_NTPCLIENT_TIMEOUT_MS**: Network timeout in milliseconds

Usage
=====

The NTP client is typically used through the system commands:

.. note:: The NTP client functionality requires enabling the :code:`SYSTEM_NTPC` option in your configuration.
    Make sure to select this option in menuconfig or your Kconfig fragment before building.

- **ntpcstart**: Start the NTP daemon
- **ntpcstop**: Stop the NTP daemon
- **ntpcstatus**: Display synchronization status

Example workflow:

1. Configure network connectivity
2. Start NTP client: ``ntpcstart``
3. Check status: ``ntpcstatus``
4. Verify time: ``date`` command
5. Stop when needed: ``ntpcstop``

The client will automatically:
- Connect to configured NTP servers
- Exchange time information
- Calculate and apply clock corrections
- Continue periodic synchronization

Limitations
===========

This is a minimal NTP client implementation with some limitations:

- **No Authentication**: Does not support NTP authentication (MD5/SHA1)
- **Basic Filtering**: Uses simple statistical filtering, not full NTP algorithms
- **Single Reference**: Does not implement full NTP reference clock selection
- **No Leap Seconds**: Does not handle leap second announcements
- **Limited Error Handling**: Basic error recovery and retry mechanisms

Despite these limitations, the implementation provides sufficient accuracy
for most embedded applications requiring network time synchronization.

Dependencies
============

The NTP client requires:

- **CONFIG_NET**: Network support
- **CONFIG_NET_UDP**: UDP protocol support
- **CONFIG_NET_SOCKOPTS**: Socket options support
- **CONFIG_LIBC_NETDB**: DNS resolution (recommended)
- **CONFIG_HAVE_LONG_LONG**: 64-bit integer support

For best results, ensure:
- Stable network connectivity
- Access to reliable NTP servers
- Sufficient system resources for daemon operation
