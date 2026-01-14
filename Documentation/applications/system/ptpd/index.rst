============================
``ptpd`` PTP daemon commands
============================
Overview
========

The ``ptpd`` application provides a complete IEEE 1588-2008 Precision Time
Protocol (PTPv2) implementation for NuttX. This daemon enables sub-microsecond
time synchronization across networked systems, supporting both client (slave)
and server (master) modes.

The PTP daemon can synchronize the system clock to a remote PTP master with
accuracy better than 1 microsecond when using software timestamps, or better
than 500 nanoseconds with hardware timestamping support.

Features
--------

- **IEEE 1588-2008 PTPv2 compliant** implementation
- **Dual role support**: operates as both PTP master and slave
- **Multiple transport options**:

  - UDP over IPv4 (default)
  - UDP over IPv6
  - IEEE 802.3 Ethernet (Layer 2)

- **Hardware timestamping** support for enhanced accuracy
- **PTP clock device integration** via ``/dev/ptp*`` devices
- **BMCA (Best Master Clock Algorithm)** for automatic master selection
- **Two delay mechanisms**:

  - End-to-End (E2E) delay measurement
  - Peer-to-Peer (P2P) delay measurement

- **gPTP support** with switch path delay correction
- **Clock drift compensation** with automatic frequency adjustment
- **Client-only mode** for dedicated slave operation

Architecture
============

The ptpd implementation consists of two main components:

Upper Layer (``netutils/ptpd/``)
---------------------------------

Provides the core PTP protocol implementation:

- PTP packet encoding/decoding (Announce, Sync, Follow-Up, Delay_Req, Delay_Resp)
- BMCA (Best Master Clock Algorithm) for master selection
- Clock synchronization and adjustment algorithms
- Path delay measurement and compensation
- Network socket management (multicast, unicast)
- Hardware and software timestamping support

Lower Layer (``system/ptpd/``)
-------------------------------

Provides the command-line interface and daemon management:

- Command-line argument parsing
- Configuration management
- Daemon lifecycle control (start, stop, status)
- Status reporting and monitoring

Configuration Options
=====================

The PTP daemon can be configured through Kconfig options in
``apps/netutils/ptpd/Kconfig``. Key configuration parameters include:

Domain and Priority
-------------------

``CONFIG_NETUTILS_PTPD_DOMAIN`` (default: 0)
  PTP domain number (0-127). Isolates different PTP domains on the same network.

``CONFIG_NETUTILS_PTPD_PRIORITY1`` (default: 128)
  Primary priority field (0-255). Lower value = higher priority in BMCA.

``CONFIG_NETUTILS_PTPD_PRIORITY2`` (default: 128)
  Secondary priority field (0-255). Used when priority1 values are equal.

Clock Quality
-------------

``CONFIG_NETUTILS_PTPD_CLASS`` (default: 248)
  Clock class value (0-255):

  - 6: Primary reference (e.g., GPS)
  - 13: Application-specific time source
  - 52: Degraded mode
  - 248: Default (unknown)

``CONFIG_NETUTILS_PTPD_ACCURACY`` (default: 254)
  Clock accuracy on logarithmic scale:

  - 32: ±25 ns
  - 35: ±1 μs
  - 39: ±100 μs
  - 41: ±1 ms
  - 47: ±1 s
  - 254: Unknown

``CONFIG_NETUTILS_PTPD_CLOCKSOURCE`` (default: 160)
  Time source type:

  - 32: GPS
  - 64: PTP
  - 80: NTP
  - 160: Internal oscillator

Timing Parameters
-----------------

``CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MSEC`` (default: 1000)
  Interval between Sync messages when acting as master (milliseconds).

``CONFIG_NETUTILS_PTPD_ANNOUNCE_INTERVAL_MSEC`` (default: 10000)
  Interval between Announce messages when acting as master (milliseconds).

``CONFIG_NETUTILS_PTPD_TIMEOUT_MS`` (default: 60000)
  Timeout for switching to alternate clock source (milliseconds).

Adjustment Thresholds
---------------------

``CONFIG_NETUTILS_PTPD_SETTIME_THRESHOLD_MS`` (default: 1000)
  Clock offset threshold for using ``settimeofday()`` instead of ``adjtime()``.
  If offset exceeds this value, time is stepped rather than slewed.

``CONFIG_NETUTILS_PTPD_ADJTIME_THRESHOLD_NS`` (default: 500)
  Threshold for using current PPB instead of accumulated PPB to accelerate
  adjustment (nanoseconds).

``CONFIG_NETUTILS_PTPD_DRIFT_AVERAGE_S`` (default: 600)
  Time period for averaging clock drift rate (seconds, 10-86400).

Path Delay
----------

``CONFIG_NETUTILS_PTPD_MAX_PATH_DELAY_NS`` (default: 100000)
  Maximum acceptable path delay (nanoseconds). Longer delays are ignored.

``CONFIG_NETUTILS_PTPD_DELAYREQ_AVGCOUNT`` (default: 100)
  Number of samples for path delay averaging.

Command Line Interface
======================

Usage
-----

.. code-block:: console

   ptpd [options]

The daemon must be run in background mode using the ``&`` operator.

Options
-------

**Mode Selection**

``-s``
  Enable client-only mode (slave only, no master capability)

**Network Transport**

``-2``
  Use IEEE 802.3 Ethernet transport (Layer 2, raw sockets)

``-4``
  Use UDP over IPv4 (default)

``-6``
  Use UDP over IPv6

**Time Stamping**

``-H``
  Use hardware timestamping (default if ``CONFIG_NET_TIMESTAMP`` is enabled)

``-S``
  Use software timestamping (fallback mode)

**Protocol Options**

``-B``
  Enable Best Master Clock Algorithm messages

``-E``
  Use End-to-End (E2E) delay mechanism (supports Delay_Req/Delay_Resp)

``-r``
  Synchronize system realtime clock (instead of PTP clock device)

**Device Configuration**

``-i [device]``
  Network interface to use (e.g., ``eth0``, ``eth1``)

``-p [device]``
  PTP clock device to use (e.g., ``ptp0``). If not specified, uses ``realtime``.

**Daemon Control**

``-t [pid]``
  Query and display status of running PTP daemon

``-d [pid]``
  Stop PTP daemon with given PID

Examples
========

Start as PTP Client (Slave)
----------------------------

Basic client synchronization using IPv4 UDP:

.. code-block:: console

   nsh> ptpd -i eth0 &
   [PTP] Starting ptpd on interface eth0
   [PTP] Operating in client+server mode
   [PTP] Using software timestamps

Client-only mode with hardware timestamping:

.. code-block:: console

   nsh> ptpd -s -H -i eth0 &
   [PTP] Starting ptpd on interface eth0
   [PTP] Client-only mode
   [PTP] Using hardware timestamps

Start as PTP Master (Server)
-----------------------------

Act as PTP master using BMCA:

.. code-block:: console

   nsh> ptpd -B -i eth0 &
   [PTP] Starting ptpd on interface eth0
   [PTP] BMCA enabled

Layer 2 Ethernet Transport
---------------------------

Use IEEE 802.3 Ethernet transport for gPTP:

.. code-block:: console

   nsh> ptpd -2 -i eth0 &
   [PTP] Starting ptpd on interface eth0
   [PTP] Using Ethernet transport (raw socket)

PTP Clock Device Integration
-----------------------------

Synchronize PTP hardware clock instead of system clock:

.. code-block:: console

   nsh> ptpd -i eth0 -p ptp0 &
   [PTP] Starting ptpd on interface eth0
   [PTP] Using PTP clock device /dev/ptp0

Query Daemon Status
-------------------

Check synchronization status of running daemon:

.. code-block:: console

   nsh> ptpd -t 42
   PTPD (PID 42) status:
   - clock_source_valid: 1
   |- id: 00 1a 2b 3c 4d 5e 6f 70
   |- utcoffset: 37
   |- priority1: 128
   |- class: 6
   |- accuracy: 32
   |- variance: 4321
   |- priority2: 128
   |- gm_id: 00 1a 2b 3c 4d 5e 6f 70
   |- stepsremoved: 0
   '- timesource: 32
   - last_clock_update: 2025-12-15T10:30:45.123456789
   - last_delta_ns: 234
   - last_adjtime_ns: -123
   - drift_ppb: 1234
   - path_delay_ns: 5678
   - last_received_multicast: 0 s ago
   - last_received_announce: 2 s ago
   - last_received_sync: 0 s ago

Stop Daemon
-----------

Stop running PTP daemon:

.. code-block:: console

   nsh> ptpd -d 42
   Stopped ptpd

Status Information
==================

When querying status with ``-t [pid]``, the following information is displayed:

Clock Source Information
------------------------

- **clock_source_valid**: Whether a valid PTP master has been selected
- **id**: PTP clock identity (EUI-64 format)
- **utcoffset**: Offset between TAI and UTC (seconds)
- **priority1/priority2**: BMCA priority fields
- **class**: Clock class (6=GPS, 13=application, 248=default)
- **accuracy**: Accuracy specification (32=±25ns, 254=unknown)
- **variance**: Clock variance estimate
- **gm_id**: Grandmaster clock identity
- **stepsremoved**: Hops from grandmaster
- **timesource**: Source type (32=GPS, 64=PTP, 80=NTP, 160=internal)

Synchronization Status
-----------------------

- **last_clock_update**: Timestamp of last clock adjustment
- **last_delta_ns**: Latest measured clock offset (nanoseconds)
- **last_adjtime_ns**: Previously applied adjustment offset
- **drift_ppb**: Averaged clock drift rate (parts per billion)
- **path_delay_ns**: Network path delay estimate (nanoseconds)

Activity Timestamps
-------------------

Time since last received/transmitted PTP message:

- **last_received_multicast**: Any PTP multicast packet
- **last_received_announce**: Announce message from any master
- **last_received_sync**: Sync message from selected master
- **last_transmitted_sync**: Sync message (when acting as master)
- **last_transmitted_announce**: Announce message (when acting as master)
- **last_transmitted_delayresp**: Delay response (when acting as master)
- **last_transmitted_delayreq**: Delay request (when acting as slave)

API Functions
=============

The ptpd library provides C API functions for programmatic control:

ptpd_start()
------------

.. code-block:: c

   int ptpd_start(FAR const struct ptpd_config_s *config);

Start PTP daemon with specified configuration.

**Parameters:**

- ``config``: Pointer to configuration structure

**Returns:**

- Does not return on success (runs as daemon)
- Negative errno value on error

**Configuration Structure:**

.. code-block:: c

   struct ptpd_config_s
   {
     FAR const char *interface;  /* Network interface (e.g., "eth0") */
     FAR const char *clock;       /* Clock device (e.g., "ptp0", "realtime") */
     bool client_only;            /* Client-only mode flag */
     bool hardware_ts;            /* Hardware timestamping flag */
     bool delay_e2e;              /* E2E delay mechanism flag */
     bool bmca;                   /* BMCA enable flag */
     sa_family_t af;              /* Address family (AF_INET, AF_INET6, AF_PACKET) */
   };

ptpd_status()
-------------

.. code-block:: c

   int ptpd_status(pid_t pid, FAR struct ptpd_status_s *status);

Query status of running PTP daemon.

**Parameters:**

- ``pid``: Process ID of ptpd daemon
- ``status``: Pointer to status structure to fill

**Returns:**

- ``OK`` on success
- Negative errno value on error

ptpd_stop()
-----------

.. code-block:: c

   int ptpd_stop(pid_t pid);

Stop running PTP daemon.

**Parameters:**

- ``pid``: Process ID of ptpd daemon to stop

**Returns:**

- ``OK`` on success
- Negative errno value on error

Implementation Details
======================

Synchronization Algorithm
--------------------------

The PTP daemon uses a multi-stage synchronization approach:

1. **Clock Selection (BMCA)**

   - Evaluates Announce messages from all masters
   - Selects best clock source based on priority, class, accuracy
   - Switches sources if current master times out

2. **Offset Measurement**

   - Measures clock offset using Sync and Follow-Up messages
   - Compensates for network path delay using Delay_Req/Delay_Resp
   - Averages measurements to reduce jitter

3. **Clock Adjustment**

   - Small offsets (<1ms): uses ``adjtime()`` for smooth slewing
   - Large offsets (>1ms): uses ``settimeofday()`` for immediate step
   - Tracks clock drift rate and applies frequency compensation

4. **Path Delay Measurement**

   - Continuously measures round-trip network delay
   - Averages over configurable sample count
   - Detects and rejects outliers (>MAX_PATH_DELAY)

Hardware Timestamping
---------------------

When hardware timestamping is enabled (``-H`` option):

- TX timestamps captured at MAC layer on packet transmission
- RX timestamps captured at MAC layer on packet reception
- Eliminates kernel and network stack processing delays
- Achieves sub-microsecond synchronization accuracy

Requires network driver support for:

- ``SO_TIMESTAMPNS`` socket option
- ``MSG_TRUNC`` flag in ``recvmsg()``
- ``SCM_TIMESTAMPNS`` control message

PTP Clock Device Support
-------------------------

When using PTP clock devices (``/dev/ptp*``):

- Synchronizes hardware PTP clock instead of system clock
- Uses ``clock_adjtime()`` via CLOCKFD mechanism
- Supports frequency and phase adjustments
- Maintains separation between system and PTP time domains

Requires kernel support for:

- ``CONFIG_PTP_CLOCK`` in NuttX kernel
- PTP clock driver implementation
- ``CONFIG_CLOCK_ADJTIME`` system call

Transport Modes
---------------

**UDP IPv4 (default)**

- Multicast address: 224.0.1.129
- Event port: 319, General port: 320
- Firewall-friendly, router-compatible

**UDP IPv6**

- Multicast address: FF0E::181
- Same port numbers as IPv4
- IPv6-only network support

**IEEE 802.3 Ethernet**

- Multicast MAC: 01:1B:19:00:00:00 (PTP) or 01:80:C2:00:00:0E (gPTP)
- EtherType: 0x88F7
- No IP/UDP overhead
- Required for gPTP compliance

Performance Considerations
==========================

Synchronization Accuracy
------------------------

Typical accuracy achievable:

- **Software timestamps**: 10-100 μs
- **Hardware timestamps**: 100-500 ns
- **With PTP clock device**: 50-200 ns

Factors affecting accuracy:

- Network jitter and asymmetry
- Interrupt latency
- Clock crystal quality
- Temperature variations

CPU and Memory Usage
--------------------

- **CPU overhead**: ~1-3% on ARM Cortex-M4 @ 168MHz
- **Memory footprint**: ~45KB code, ~8KB RAM
- **Network bandwidth**: ~10-20 packets/second (typical)

Network Requirements
--------------------

- **Multicast support**: Required for PTP operation
- **IGMP**: Must be enabled for IPv4 multicast
- **Switch compatibility**: PTP-aware switches recommended for best results
- **Bandwidth**: Minimal (<100 Kbps)

Troubleshooting
===============

Common Issues
-------------

**No synchronization occurring**

- Check network interface is up and has IP address
- Verify multicast routing is enabled
- Ensure firewall allows UDP ports 319/320
- Check PTP master is on same network/VLAN

**Large clock offsets**

- Verify network path delay is reasonable (<100μs typical)
- Check for asymmetric network delays
- Enable hardware timestamping if available
- Review BMCA priority settings

**Frequent master switches**

- Increase ``CONFIG_NETUTILS_PTPD_TIMEOUT_MS``
- Check network stability and packet loss
- Verify all masters have unique clock identities

**High CPU usage**

- Increase sync interval (``CONFIG_NETUTILS_PTPD_SYNC_INTERVAL_MSEC``)
- Use hardware timestamping to reduce processing
- Check for excessive packet errors/retries

Debug Output
------------

Enable PTP debug messages in kernel configuration:

- ``CONFIG_DEBUG_PTP_ERROR``: Error messages
- ``CONFIG_DEBUG_PTP_WARN``: Warning messages
- ``CONFIG_DEBUG_PTP_INFO``: Informational messages

Monitor debug output:

.. code-block:: console

   nsh> dmesg | grep PTP

Related Documentation
=====================

- :doc:`/components/drivers/special/ptp` - PTP Clock Driver Framework
- :doc:`/applications/netutils/index` - Network Utilities Overview
- IEEE 1588-2008 Standard - Precision Time Protocol specification
- IEEE 802.1AS Standard - Timing and Synchronization for Time-Sensitive Applications (gPTP)

References
==========

- IEEE 1588-2008: "IEEE Standard for a Precision Clock Synchronization Protocol for Networked Measurement and Control Systems"
- IEEE 802.1AS-2020: "IEEE Standard for Local and Metropolitan Area Networks - Timing and Synchronization for Time-Sensitive Applications"
- Linux PTP Project: https://linuxptp.sourceforge.net/
- ``apps/netutils/ptpd/`` - PTP protocol implementation
- ``apps/system/ptpd/`` - PTP daemon command interface
- ``include/netutils/ptpd.h`` - PTP API header
