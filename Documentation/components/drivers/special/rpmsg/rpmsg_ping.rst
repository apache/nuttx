RPMsg Ping
==========

Overview
--------

RPMsg Ping is a diagnostic tool for testing inter-processor communication (IPC)
functionality and measuring performance metrics such as latency and throughput.
It is useful for:

- Verifying RPMsg IPC works correctly during early project development
- Evaluating RPMsg IPC performance (latency and bandwidth)
- Debugging communication issues between cores

Source Files
------------

::

    nuttx/
    ├── drivers/
    │   └── rpmsg/
    │       ├── rpmsg_ping.c
    │       └── rpmsg_ping.h
    └── include/
        └── nuttx/
            └── rpmsg/
                └── rpmsg_ping.h

Configuration
-------------

Enable the following configuration on **both** communicating cores:

.. code-block:: makefile

   CONFIG_RPMSG_PING=y

Usage
-----

Command Syntax
~~~~~~~~~~~~~~

.. code-block:: bash

   rpmsg ping <path> <times> <length> <ack> <period(ms)>

Parameters
~~~~~~~~~~

``<path>``
    The rptun device path. For example, to ping a CPU named "remote",
    use ``/dev/rpmsg/remote``.

``<times>``
    Number of ping iterations.

``<length>``
    Payload size in bytes for each ping packet.

``<ack>``
    Bitmask controlling ping behavior:

    - **Bit 0**: Wait for acknowledgment before next ping (1=yes, 0=no)
    - **Bit 1**: Verify data integrity
    - **Bit 2**: Use random payload length

``<period(ms)>``
    Interval between consecutive pings in milliseconds.

Examples
~~~~~~~~

**Example 1: Basic ping without acknowledgment**

.. code-block:: bash

   rpmsg ping /dev/rpmsg/remote 50 100 0 1000

Ping the "remote" CPU 50 times with 100-byte packets, no acknowledgment required,
default endpoint priority, 1000ms interval.

**Example 2: Ping with acknowledgment**

.. code-block:: bash

   rpmsg ping /dev/rpmsg/ap 1000 100 1 10

Ping the "ap" CPU 1000 times with 100-byte packets, wait for acknowledgment,
default endpoint priority, 10ms interval.

Output
------

**From AP core pinging remote core:**

.. code-block:: bash

   ap> rpmsg ping /dev/rpmsg/remote 100 100 1 100 &
   rpmsg [26:254]
   ap> [   44.250700] [26] [  INFO] [ap] current CPU freq: 1000000, ping times: 100
   [   44.251000] [26] [  INFO] [ap] avg: s 0, ns 11102000
   [   44.251100] [26] [  INFO] [ap] min: s 0, ns 10182000
   [   44.251200] [26] [  INFO] [ap] max: s 0, ns 11630000

**From remote core pinging AP core:**

.. code-block:: bash

   remote> rpmsg ping /dev/rpmsg/ap 100 100 1 100 &
   rpmsg [6:100]
   remote> [  175.584500] [remote] current CPU freq: 1000000, ping times: 100
   [  175.584700] [remote] avg: s 0, ns 11102000
   [  175.584800] [remote] min: s 0, ns 10017000
   [  175.585000] [remote] max: s 0, ns 11244000

The output shows:

- **avg**: Average round-trip latency (seconds, nanoseconds)
- **min**: Minimum round-trip latency
- **max**: Maximum round-trip latency

Architecture
------------

Workflow
~~~~~~~~

The following diagram illustrates the complete ping workflow between two cores:

::

    ┌─────────────────────────────────────────────────────────────────────────┐
    │                           Ping Execution                                │
    ├─────────────────────────────────┬───────────────────────────────────────┤
    │            Core A               │              Core B                   │
    │                                 │                                       │
    │  NSH: rpmsg ping /dev/rpmsg/B   │                                       │
    │         │                       │                                       │
    │         ▼                       │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │  cmd_rpmsg()    │            │                                       │
    │  │  Parse args     │            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │ ioctl()         │            │                                       │
    │  │    │            │            │                                       │
    │  │    ▼            │            │                                       │
    │  │ VFS layer       │            │                                       │
    │  │    │            │            │                                       │
    │  │    ▼            │            │                                       │
    │  │ rpmsg_ioctl()   │            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │  rpmsg_ping()   │            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │rpmsg_ping_once()│            │                                       │
    │  │                 │            │                                       │
    │  │ 1. get_tx_      │            │                                       │
    │  │    payload_     │            │                                       │
    │  │    buffer()     │            │                                       │
    │  │                 │            │                                       │
    │  │ 2. Fill msg:    │            │                                       │
    │  │    cmd, len,    │            │                                       │
    │  │    cookie, data │            │                                       │
    │  │                 │            │                                       │
    │  │ 3. send_nocopy()│            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           │   PING_CMD_REQ      │                                       │
    │           └─────────────────────┼──────────────────┐                    │
    │                                 │                  ▼                    │
    │                                 │  ┌─────────────────────────────────┐  │
    │                                 │  │     rpmsg_ping_ept_cb()         │  │
    │                                 │  │                                 │  │
    │                                 │  │  1. Receive ping request        │  │
    │                                 │  │                                 │  │
    │                                 │  │  2. Check data (if CHECK_MASK)  │  │
    │                                 │  │                                 │  │
    │                                 │  │  3. If ACK_MASK set:            │  │
    │                                 │  │     - Set cmd = PING_CMD_RSP    │  │
    │                                 │  │     - rpmsg_send() response     │  │
    │                                 │  └──────────────┬──────────────────┘  │
    │                                 │                 │                     │
    │           ┌─────────────────────┼─────────────────┘                     │
    │           │   PING_CMD_RSP      │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │rpmsg_ping_ept_cb│            │                                       │
    │  │                 │            │                                       │
    │  │ nxsem_post(sem) │            │                                       │
    │  │ (wake up wait)  │            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │ Calculate stats │            │                                       │
    │  │ min/max/avg     │            │                                       │
    │  └────────┬────────┘            │                                       │
    │           │                     │                                       │
    │           ▼                     │                                       │
    │  ┌─────────────────┐            │                                       │
    │  │ Output results  │            │                                       │
    │  │ avg/min/max/rate│            │                                       │
    │  └─────────────────┘            │                                       │
    └─────────────────────────────────┴───────────────────────────────────────┘

Detailed Flow Description
~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Initialization**: When RPMsg initializes, both cores call ``rpmsg_ping_init()``
   to create an endpoint named ``rpmsg-ping`` with callback ``rpmsg_ping_ept_cb()``.
   See ``nuttx/drivers/rpmsg/rpmsg_ping.c``.

2. **Ping Start**: User runs ``rpmsg ping ...`` in NSH. The ``cmd_rpmsg()`` function
   parses command-line arguments, then calls through:
   ``ioctl()`` → VFS → ``rpmsg_ioctl()`` → ``rpmsg_ping()``.

3. **Ping Logic**: ``rpmsg_ping_once()`` implements single ping iteration:

   - ``rpmsg_get_tx_payload_buffer()`` - acquire transmit buffer (zero-copy)
   - Fill message structure (cmd, len, cookie, data)
   - ``rpmsg_send_nocopy()`` - send ping packet

4. **Receive and Response**: When Core B receives the ping packet,
   ``rpmsg_ping_ept_cb()`` handles it:

   - If ``RPMSG_PING_CHECK_MASK`` is set, verify data integrity
   - If ``RPMSG_PING_ACK_MASK`` is set, send response with ``RPMSG_PING_CMD_RSP``

5. **Response Handling**: Core A's ``rpmsg_ping_ept_cb()`` receives the response
   and calls ``nxsem_post()`` to wake up the waiting ``rpmsg_ping_once()``

6. **Statistics**: After all iterations complete, ``rpmsg_ping()`` outputs
   average, minimum, maximum latency and throughput rate

Best Practices
--------------

Latency Testing
~~~~~~~~~~~~~~~

- Use more ping iterations for accurate average latency measurements
- Recommended: 100+ iterations for statistical significance

Throughput Testing
~~~~~~~~~~~~~~~~~~

- Larger packet sizes generally yield higher throughput
- Adjust ``<length>`` parameter based on your use case requirements
