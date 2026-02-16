RPMsg Dump
==========

Overview
--------

RPMsg Dump is a diagnostic command for dumping RPMsg debugging information.
When troubleshooting inter-processor communication issues, this tool provides
detailed information about RPMsg instances, endpoints, virtqueues, and buffer
states to assist in debugging.

This document primarily describes the dump content for **RPMsg VirtIO** transport,
which is the most commonly used transport layer.

Configuration
-------------

Enable the following configuration:

.. code-block:: makefile

   CONFIG_RPMSG=y

Usage
-----

Command Syntax
~~~~~~~~~~~~~~

.. code-block:: bash

   rpmsg dump <path>

Parameters
~~~~~~~~~~

``<path>``
    The rpmsg device path. For example, ``/dev/rpmsg/remote``.

Example Output
--------------

.. code-block:: bash

   [12/31 00:01:25] [ 0] [ap] Local: ap Remote: cp Headrx 550 Headtx 504
   [12/31 00:01:25] [ 0] [ap] Dump rpmsg info between cpu (master: yes)ap <=> cp:
   [12/31 00:01:25] [ 0] [ap]   rpmsg ept list:
   [12/31 00:01:25] [ 0] [ap]     ept NS 0x3c4784d0: addr=53 dest=53 refcnt=1 priority=127 priv=0x3c4784c8
   [12/31 00:01:25] [ 0] [ap]     ept rpmsg-rtc 0x3c3e3428: addr=1024 dest=1033 refcnt=1 priority=127 priv=0x3c37af90
   [12/31 00:01:25] [ 0] [ap]     ept rpmsg-ttyCP 0x3c379e40: addr=1025 dest=1026 refcnt=1 priority=127 priv=0x3c374a00
   [12/31 00:01:25] [ 0] [ap]     ept rpmsg-sensor 0x3c3f4ac8: addr=1028 dest=1028 refcnt=1 priority=127 priv=0x3c3f4ac0
   [12/31 00:01:25] [ 0] [ap]     ept rpmsg-ping 0x3c478418: addr=1029 dest=1031 refcnt=1 priority=127 priv=0
   [12/31 00:01:25] [ 0] [ap]     ept rpmsg-syslog 0x3c3e5818: addr=1034 dest=1025 refcnt=1 priority=127 priv=0x3c3e5810
   [12/31 00:01:25] [ 0] [ap] rpmsg vq RX:
   [12/31 00:01:25] [ 0] [ap] VQ: rx_vq - size=16; free=0; queued=0; desc_head_idx=32768; available_idx=0; avail.idx=566; used_cons_idx=550; used.idx=550; avail.flags=0x0; used.flags=0x0
   [12/31 00:01:25] [ 0] [ap] rpmsg vq TX:
   [12/31 00:01:25] [ 0] [ap] VQ: tx_vq - size=16; free=6; queued=0; desc_head_idx=10; available_idx=0; avail.idx=504; used_cons_idx=494; used.idx=504; avail.flags=0x0; used.flags=0x0
   [12/31 00:01:25] [ 0] [ap]   rpmsg buffer list:
   [12/31 00:01:25] [ 0] [ap]     RX buffer total 16
   [12/31 00:01:25] [ 0] [ap]       unretrieved 0
   [12/31 00:01:25] [ 0] [ap]       retrieved 0
   [12/31 00:01:25] [ 0] [ap]       pending 0:
   [12/31 00:01:25] [ 0] [ap]     TX buffer total 16
   [12/31 00:01:25] [ 0] [ap]       unretrieved 10
   [12/31 00:01:25] [ 0] [ap]       retrieved 6
   [12/31 00:01:25] [ 0] [ap]       sent 0:

Output Description
------------------

CPU Information
~~~~~~~~~~~~~~~

The first section shows basic CPU information:

.. code-block:: bash

   Local: ap Remote: cp Headrx 550 Headtx 504
   Dump rpmsg info between cpu (master: yes)ap <=> cp:

- **Local/Remote**: Names of the local and remote CPUs
- **master**: Indicates whether the current CPU is the master in this RPMsg connection
- **Headrx/Headtx**: Counters used for debugging interrupt loss issues
  (compared with Virtqueue indices in the Virtqueue dump section)

Endpoint Information
~~~~~~~~~~~~~~~~~~~~

This section lists all endpoints maintained by the current RPMsg instance:

.. code-block:: bash

   rpmsg ept list:
     ept NS 0x3c4784d0: addr=53 dest=53 refcnt=1 priority=127 priv=0x3c4784c8
     ept rpmsg-ttyCP 0x3c379e40: addr=1025 dest=1026 refcnt=1 priority=127 priv=0x3c374a00
     ept rpmsg-sensor 0x3c3f4ac8: addr=1028 dest=1028 refcnt=1 priority=127 priv=0x3c3f4ac0

Each endpoint entry contains:

- **Endpoint Name**: The service name (e.g., ``rpmsg-ttyCP``, ``rpmsg-sensor``)
- **Pointer**: Memory address of the endpoint structure
- **addr**: Local endpoint address
- **dest**: Remote endpoint address (``4294967295`` or ``0xFFFFFFFF`` means not connected)
- **refcnt**: Reference count
- **priority**: Endpoint priority (0-255, higher value = higher priority)
- **priv**: Private data pointer

Virtqueue Dump
~~~~~~~~~~~~~~

This section dumps the core data structures of the Virtqueue:

.. code-block:: bash

   rpmsg vq RX:
   VQ: rx_vq - size=16; free=0; queued=0; desc_head_idx=32768; available_idx=0; avail.idx=566; used_cons_idx=550; used.idx=550; avail.flags=0x0; used.flags=0x0
   rpmsg vq TX:
   VQ: tx_vq - size=16; free=6; queued=0; desc_head_idx=10; available_idx=0; avail.idx=504; used_cons_idx=494; used.idx=504; avail.flags=0x0; used.flags=0x0

Key fields:

- **size**: Total number of descriptors in the virtqueue
- **free**: Number of free descriptors
- **queued**: Number of queued descriptors
- **avail.idx**: Available ring index (producer side)
- **used.idx**: Used ring index (consumer side)
- **used_cons_idx**: Consumer's view of used ring index

.. note::

   Generally, you don't need to analyze this section directly.
   The important information is summarized in the RPMsg Buffer List section below.

RPMsg Buffer List
~~~~~~~~~~~~~~~~~

This section provides a summary of RX/TX buffer states:

.. code-block:: bash

   rpmsg buffer list:
     RX buffer total 16
       unretrieved 0
       retrieved 0
       pending 0:
     TX buffer total 16
       unretrieved 10
       retrieved 6
       sent 0:

**RX Buffer States:**

- **total**: Total number of RX buffers
- **unretrieved**: Buffers received and in RX queue, not yet retrieved
- **retrieved**: Buffers retrieved from RX queue but not yet returned
- **pending**: Buffers pending processing

**TX Buffer States:**

- **total**: Total number of TX buffers
- **unretrieved**: Number of free TX buffers available
- **retrieved**: Buffers retrieved from free pool but not yet sent
- **sent**: Buffers sent but not yet returned by remote side

Debugging Use Cases
-------------------

TX Buffer Timeout Issues
~~~~~~~~~~~~~~~~~~~~~~~~

When a thread fails to acquire a TX buffer (timeout), check:

1. **Remote RX thread state**: Is it blocked on a semaphore/mutex (possible deadlock)?
2. **Remote RX thread ready but not running**: Check CPU load - higher priority tasks
   may be starving the RX thread
3. **Remote RX thread waiting but RX queue has data**: Compare ``Headrx`` with
   virtqueue indices to check for interrupt loss

Communication Not Working
~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Check if RPMsg thread exists**: Use ``ps`` command to verify thread presence
2. **Check if RPMsg instance exists**: List ``/dev/rpmsg/`` directory
3. **Use RPMsg Ping**: Test basic connectivity with ``rpmsg ping`` command
4. **Check endpoint dest address**: If ``dest=4294967295``, the endpoint is not
   connected to the remote side

Buffer Leak Detection
~~~~~~~~~~~~~~~~~~~~~

If ``retrieved`` count keeps increasing over time without corresponding
``unretrieved`` increase, there may be a buffer leak where buffers are
being acquired but not properly released.

See Also
--------

- :doc:`concepts` - RPMsg core concepts
- :doc:`rpmsg_ping` - RPMsg connectivity testing tool
