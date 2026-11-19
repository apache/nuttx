=============================
TCP Urgent (Out-of-Band) Data
=============================

Overview
========

TCP Urgent Data (often designated as Out-of-Band or OOB data) allows a
transmitting endpoint to mark a specific point in the byte stream as
urgent. This mechanism informs the remote peer to process the designated byte
promptly, without waiting for previously buffered stream data to be consumed.

In Apache NuttX, support for TCP urgent data is conditionally compiled via the
``CONFIG_NET_TCPURGDATA`` configuration option. The implementation conforms to
standard BSD and POSIX socket semantics:

* **Ingress (RX) Path**: In-order segment parsing, stream separation via
  I/O buffer (IOB) compaction, out-of-order segment handling (when
  ``CONFIG_NET_TCP_OUT_OF_ORDER`` is enabled), and immediate acknowledgment
  generation.
* **Egress (TX) Path**: Urgent pointer generation for both unbuffered and
  buffered write modes (when ``CONFIG_NET_TCP_WRITE_BUFFERS`` is enabled),
  write buffer coalescing inhibition, and header field isolation.
* **Socket Interface**: Standard ``recv()``, ``recvfrom()``, and ``recvmsg()``
  support using ``MSG_OOB`` and ``MSG_PEEK`` flags, configurable inline
  delivery via the ``SO_OOBINLINE`` socket option, and cancellation-safe
  blocking operations.
* **Event Multiplexing**: Asynchronous readiness notification via ``poll()``
  and ``select()`` using the ``POLLPRI`` event mask.

The Out-of-Band Mechanism
-------------------------

Under default TCP stream semantics, data is delivered in a strictly ordered,
first-in, first-out (FIFO) sequence. If an application queues a large block
of bulk data, subsequent control messages (such as an interrupt or abort
signal in interactive protocols like Telnet or FTP) would normally remain
queued until all prior bytes are received and read.

The TCP urgent mechanism addresses this by asserting the ``TCP_URG`` control
bit in the TCP header and specifying a 16-bit **Urgent Pointer** (``urgp``).
Upon receiving a segment with ``TCP_URG`` set, the network stack marks the
presence of urgent data, notifies waiting threads or polling contexts via
``POLLPRI``, and stores the urgent byte for retrieval via ``recv(..., MSG_OOB)``.

Standards and Semantics
=======================

Urgent Pointer Interpretation
-----------------------------

Historical standards differ on how the TCP urgent pointer field is calculated:

* **RFC 793**: States that the urgent pointer represents a positive offset from
  the segment sequence number pointing to the sequence number of the urgent
  octet itself.
* **BSD Convention / RFC 1122 / RFC 6093**: Standard BSD implementations, as
  well as Linux by default (``tcp_stdurg=0``), interpret the urgent pointer
  as pointing to the octet *immediately following* the urgent data. Under this
  convention, the urgent byte is located at sequence number:

  .. code-block:: text

      urgseq = seq + urgp - 1

  Or at payload index:

  .. code-block:: text

      payload[urgp - 1]

Apache NuttX implements the **BSD / Linux default convention** (``tcp_stdurg=0``)
to maintain compatibility with modern operating systems and network stacks.
An incoming segment with ``urgp == 0`` is treated as invalid under this
interpretation (as ``urgp - 1`` would yield sequence ``seq - 1``) and is
ignored.

Single-Byte Bounded Storage
---------------------------

Per BSD conventions, NuttX maintains at most **one** unread urgent data byte
per TCP connection. The state is maintained within ``struct tcp_conn_s``
(declared in ``net/tcp/tcp.h``):

.. code-block:: c

  struct tcp_conn_s
  {
    ...
  #ifdef CONFIG_NET_TCPURGDATA
    uint8_t   urg_data;    /* Single buffered urgent byte */
    uint8_t   urg_state;   /* Lifecycle state (TCP_URG_*) */
    uint16_t  urg_offset;  /* Outgoing urgent pointer offset for TX */
    uint32_t  urg_seq;     /* Absolute sequence number of urgent byte */
  #endif
    ...
  };

Characteristics of this design include:

* **Static Allocation**: Processing urgent data requires no dynamic heap
  allocations (``malloc``).
* **Buffer Resource Protection**: Senders transmitting frequent urgent
  notifications cannot exhaust the shared I/O buffer (IOB) pool.
* **Memory Footprint**: Adding ``CONFIG_NET_TCPURGDATA`` increases
  ``struct tcp_conn_s`` by exactly 8 bytes (two 8-bit integers, one 16-bit
  integer, and one 32-bit integer). If ``CONFIG_NET_TCP_WRITE_BUFFERS`` is
  also enabled, ``struct tcp_wrbuffer_s`` adds 2 bytes for ``wb_urg``.

Stream Separation vs. SO_OOBINLINE
----------------------------------

* **Default Mode (SO_OOBINLINE disabled)**:
  The urgent byte is removed from the normal stream payload using
  ``tcp_remove_urgbyte()``. The extracted byte is copied into
  ``conn->urg_data`` and can be read only by calling ``recv()``,
  ``recvfrom()``, or ``recvmsg()`` with the ``MSG_OOB`` flag. Routine
  ``read()`` or ``recv()`` calls without ``MSG_OOB`` will not encounter the
  urgent byte.
* **Inline Mode (SO_OOBINLINE enabled)**:
  When the ``SO_OOBINLINE`` socket option is enabled via ``setsockopt()``, the
  urgent byte is retained in the standard receive stream. In accordance with
  POSIX.1-2017:

  * Calling ``recv(..., MSG_OOB)`` returns ``-1`` with ``errno`` set to
    ``EINVAL``.
  * The urgent byte is read sequentially alongside routine stream data.
  * The arrival of urgent data continues to trigger ``POLLPRI`` notifications
    for ``poll()`` and ``select()``.

If ``SO_OOBINLINE`` is modified via ``setsockopt()`` while an urgent pointer
has been announced but the byte has not yet arrived (state
``TCP_URG_AWAITING``), the option value present at the moment the segment is
processed determines whether the byte is excised or retained in-line.

Configuration
=============

``CONFIG_NET_TCPURGDATA``
  Enables TCP urgent (out-of-band) data processing.

  - **Prompt**: *TCP urgent data support*
  - **Type**: boolean
  - **Default**: ``n``
  - **Dependencies**: ``CONFIG_NET_TCP``

.. note::

   Neither ``CONFIG_NET_TCP_WRITE_BUFFERS`` nor
   ``CONFIG_NET_TCP_OUT_OF_ORDER`` is a required dependency of
   ``CONFIG_NET_TCPURGDATA``. If write buffers are disabled, urgent
   transmission operates through the unbuffered send path. If out-of-order
   handling is disabled, segments arriving out of sequence are handled
   according to standard in-order TCP rules.

System Architecture and Data Flow
=================================

The following diagram illustrates the flow of TCP urgent data across the
NuttX network stack:

.. code-block:: text

   =============================================================================
                          TRANSMIT (TX) DATA FLOW
   =============================================================================

   Application Space
          |
          | send(sockfd, &cmd, 1, MSG_OOB)
          v
   Socket Layer (net/socket/send.c -> psock_send)
          |
          v
   TCP Egress Handler
     +-----------------------------------+-----------------------------------+
     |         Unbuffered Path           |           Buffered Path           |
     |   (net/tcp/tcp_send_unbuffered.c) |   (net/tcp/tcp_send_buffered.c)   |
     |                                   |                                   |
     | - Sets pstate->snd_flags=MSG_OOB  | - Buffer coalescing DISABLED      |
     | - Sets conn->urg_offset = len     | - Records wrb->wb_urg = pktlen    |
     | - tcp_appsend(..., sendflags)     | - Adjusts wb_urg on partial ACKs  |
     +-----------------+-----------------+-----------------+-----------------+
                       |                                   |
                       +-----------------+-----------------+
                                         |
                                         v
   Header Framing (net/tcp/tcp_send.c -> tcp_fill_header)
          |
          |-- Asserts TCP_URG bit in tcp->flags
          |-- Sets 16-bit offset: tcp->urgp = conn->urg_offset
          '-- Clears conn->urg_offset = 0 (prevents leakage into routine ACKs)
          |
          v
   Device Driver (dev->d_iob) ---------> [ Physical / Virtual Network Wire ]


   =============================================================================
                           RECEIVE (RX) DATA FLOW
   =============================================================================

   Device Driver (dev->d_iob) <--------- [ Physical / Virtual Network Wire ]
          |
          v
   TCP Ingress Handler (net/tcp/tcp_input.c)
          | - Validates checksum and state (ESTABLISHED, FIN_WAIT_1, FIN_WAIT_2)
          v
   Urgent Ingress Parser (tcp_urg_input)
          | - Decodes urgp = (tcp->urgp[0] << 8) | tcp->urgp[1]
          | - Verifies urgp >= 1 and TCP_SEQ_GTE(urgseq, urg_rcvseq)
          |
          +---------------------------------+
          |                                 |
          v                                 v
   +---------------------------------+   +---------------------------------+
   |   Case A: urgp <= dev->d_len    |   |    Case B: urgp > dev->d_len    |
   |  (Urgent byte in this segment)  |   | (Urgent byte in future segment) |
   |                                 |   |                                 |
   | 1. iob_copyout(&conn->urg_data) |   | 1. Record conn->urg_seq = urgseq|
   | 2. conn->urg_state = VALID      |   | 2. conn->urg_state = AWAITING   |
   | 3. If !SO_OOBINLINE:            |   | 3. Flags |= TCP_URGDATA         |
   |    tcp_remove_urgbyte() slices  |   +----------------+----------------+
   |    byte and dev->d_len--        |                    |
   | 4. If d_len == 0 (pure urgent): |                    v
   |    rcvseq++ and force SNDACK    |     Out-of-Order Segment Reassembly
   +----------------+----------------+     (net/tcp/tcp_callback.c)
                    |                      - Triggers when gap is filled
                    |                      - Extracts byte into urg_data
                    |                      - conn->urg_state = VALID
                    |                                     |
                    +------------------+------------------+
                                       |
                                       v
   Connection Event Notification (devif_conn_event -> TCP_URGDATA)
          |
          +---------------------------------+
          |                                 |
          v                                 v
   +---------------------------------+   +---------------------------------+
   |     Asynchronous Readiness      |   |     Synchronous Thread Wait     |
   |     (net/tcp/tcp_netpoll.c)     |   |    (net/tcp/tcp_recvfrom.c)     |
   |                                 |   |                                 |
   | - tcp_poll_eventhandler()       |   | - Blocked recv(MSG_OOB) wakes   |
   | - Translates TCP_URGDATA        |   | - Semaphore state.ir_sem posted |
   | - Sets POLLPRI in revents       |   | - Copies conn->urg_data to user |
   | - Unblocks poll() / select()    |   | - Sets conn->urg_state = READ   |
   +----------------+----------------+   +----------------+----------------+
                    |                                     |
                    v                                     v
   Application Space:                    Application Space:
   poll() returns with POLLPRI           recv(..., MSG_OOB) returns 1 byte

State Machine
=============

The connection tracks urgent data using four states:

.. list-table:: TCP Urgent Lifecycle States
   :header-rows: 1
   :widths: 25 10 65

   * - State
     - Value
     - Description
   * - ``TCP_URG_NOTSEEN``
     - 0
     - Initial state. No urgent data has been announced or processed.
   * - ``TCP_URG_AWAITING``
     - 1
     - An urgent pointer was announced, but the urgent payload byte has not
       yet arrived (it resides in a future segment or in the out-of-order queue).
   * - ``TCP_URG_VALID``
     - 2
     - The urgent byte has been extracted and buffered in ``conn->urg_data``.
       Available to be read via ``recv(..., MSG_OOB)``.
   * - ``TCP_URG_READ``
     - 3
     - The urgent byte has been read by the application via
       ``recv(..., MSG_OOB)`` without ``MSG_PEEK``.

State Transition Diagram
------------------------

.. code-block:: text

                         +-------------------+
                         |  TCP_URG_NOTSEEN  |
                         +-------------------+
                                   |
                  +----------------+----------------+
                  |                                 |
                  | URG announced                   | URG byte arrives
                  | (urgp > dev->d_len or           | in-order
                  |  out-of-order segment)          | (urgp <= dev->d_len)
                  v                                 |
        +-------------------+                       |
        | TCP_URG_AWAITING  |                       |
        +-------------------+                       |
                  |                                 |
                  | Awaited byte arrives            |
                  | (in-order / OFOSEG drain)       |
                  v                                 |
        +-------------------+                       |
  .---->|   TCP_URG_VALID   |<----------------------+
  |     +-------------------+
  |               |
  | New URG byte  | recv(..., MSG_OOB)
  | arrives and   | (without MSG_PEEK)
  | overwrites    v
  |     +-------------------+
  '-----|   TCP_URG_READ    |
        +-------------------+

Transition Details
------------------

1. **Overwriting Existing Data**:
   If a new urgent byte arrives while in ``TCP_URG_VALID``, the newly arrived
   byte replaces ``conn->urg_data``, ``conn->urg_seq`` is updated, and the
   state remains ``TCP_URG_VALID``.
2. **Transition from VALID to AWAITING**:
   If ``conn->urg_state`` is ``TCP_URG_VALID`` and a new segment arrives with
   an urgent pointer pointing to a future sequence number
   (``urgp > dev->d_len``), ``conn->urg_state`` transitions to
   ``TCP_URG_AWAITING`` and ``conn->urg_seq`` is updated. The previous
   unread byte is superseded.
3. **Peeking with MSG_PEEK**:
   Calling ``recv(..., MSG_OOB | MSG_PEEK)`` copies the urgent byte to the
   user buffer without altering ``conn->urg_state``.
4. **Consumption with MSG_OOB**:
   Calling ``recv(..., MSG_OOB)`` without ``MSG_PEEK`` moves the state from
   ``TCP_URG_VALID`` to ``TCP_URG_READ``.
5. **Connection Teardown**:
   If a FIN or RST segment is processed while ``conn->urg_state == TCP_URG_VALID``,
   the buffered byte remains accessible until the application closes the
   socket file descriptor. Once ``close()`` is called, the connection is freed
   and descriptor operations return ``EBADF``.

Execution Details
=================

Egress (TX) Implementation
--------------------------

Transmitting urgent data is initiated when an application passes ``MSG_OOB``
to ``send()``, ``sendto()``, or ``sendmsg()``:

1. **Unbuffered Egress (tcp_send_unbuffered.c)**:

   - The transfer state records ``pstate->snd_flags = MSG_OOB``.
   - When the final segment chunk is queued, the stack sets
     ``conn->urg_offset`` to the segment payload length.
   - ``tcp_appsend()`` checks ``conn->urg_offset`` and asserts the
     ``TCP_URG`` flag in ``sendflags``.

2. **Buffered Egress (tcp_send_buffered.c)**:

   - **Coalescing Inhibition**: When ``flags & MSG_OOB`` is set, data is
     not appended to an existing write buffer (``struct tcp_wrbuffer_s``),
     ensuring the urgent mark aligns with packet boundaries:

     .. code-block:: c

       if ((flags & MSG_OOB) == 0 && wrb != NULL && wrb->wb_urg == 0 && ...)
         {
           /* Append to existing write buffer */
         }
       else
         {
           /* Allocate fresh write buffer */
         }

   - **Offset Accounting**: The write buffer records the urgent position:

     .. code-block:: c

       wrb->wb_urg = TCP_WBPKTLEN(wrb);

   - **Acknowledgment Trimming**: As the remote peer acknowledges bytes,
     ``wrb->wb_urg`` is adjusted proportionally:

     .. code-block:: c

       if (wrb->wb_urg > 0)
         {
           if (nacked >= wrb->wb_urg)
             {
               wrb->wb_urg = 0;
             }
           else
             {
               wrb->wb_urg -= nacked;
             }
         }

3. **Header Construction (tcp_send.c -> tcp_fill_header)**:

   - When preparing the TCP header, the urgent fields are populated:

     .. code-block:: c

       if ((tcp->flags & TCP_URG) != 0 && conn->urg_offset > 0)
         {
           tcp->urgp[0] = (uint8_t)(conn->urg_offset >> 8);
           tcp->urgp[1] = (uint8_t)(conn->urg_offset & 0xff);
           conn->urg_offset = 0;
         }
       else
         {
           tcp->urgp[0] = 0;
           tcp->urgp[1] = 0;
         }

   - If ``conn->urg_offset == 0`` or ``TCP_URG`` is clear, ``tcp->urgp`` is
     set to 0 to prevent urgent pointer leakage into standard ACK frames.

Worked Retransmission Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Consider a buffered transfer with urgent data:

1. Application calls ``send(sockfd, buf, 100, MSG_OOB)``.
2. A write buffer ``wrb`` is created with ``pktlen = 100`` and ``wb_urg = 100``.
3. Due to window limits, only 60 bytes are initially sent:

   - Sent: 60 bytes.
   - Initial transmission sets ``conn->urg_offset = wrb->wb_urg = 100``.
   - ``TCP_WBSENT(wrb)`` becomes 60.

4. Peer acknowledges 60 bytes (``nacked = 60``):

   - Buffer is trimmed: ``TCP_WBTRIM(wrb, 60)``. Remaining ``pktlen = 40``.
   - Urgent offset adjusted: ``wrb->wb_urg = 100 - 60 = 40``.
   - ``TCP_WBSENT(wrb)`` resets to 0.

5. Retransmission / transmission of remaining 40 bytes:

   - In ``psock_send()``, ``conn->urg_offset = wrb->wb_urg - TCP_WBSENT(wrb) = 40 - 0 = 40``.
   - The segment carries 40 bytes with urgent pointer 40 (pointing to byte 39).
   - Once all 40 bytes are sent, ``TCP_WBSENT(wrb) == 40``. Subsequent trailing
     segments set ``conn->urg_offset = 0`` because ``wb_urg <= TCP_WBSENT(wrb)``.

Ingress (RX) Implementation
---------------------------

Packet ingress is handled by ``net/tcp/tcp_input.c`` and ``net/tcp/tcp_callback.c``:

1. **Ingress Filtering**:

   - Ingress segments are validated for TCP checksum and connection state.
   - Out-of-window retransmitted segments are trimmed via ``tcp_trim_head()``;
     if completely outside the window, they are ACKed and dropped before
     reaching urgent parsing.

2. **In-Order Urgent Parsing (tcp_urg_input)**:

   - When ``tcp->flags & TCP_URG`` is asserted:

     .. code-block:: c

       uint16_t urgp = (tcp->urgp[0] << 8) | tcp->urgp[1];

   - Under BSD convention, ``urgp >= 1`` is required. The sequence number is:

     .. code-block:: c

       uint32_t urgseq = urg_seq + urgp - 1;

   - Old duplicate segments are filtered by checking ``TCP_SEQ_GTE(urgseq, urg_rcvseq)``.
   - If ``urgp <= dev->d_len``: The urgent byte is in the current segment.
     It is copied to ``conn->urg_data`` using ``iob_copyout()``, and
     ``conn->urg_state`` becomes ``TCP_URG_VALID``.
   - If ``urgp > dev->d_len``: The urgent byte is in a future segment. The
     stack records ``conn->urg_seq = urgseq`` and sets ``conn->urg_state = TCP_URG_AWAITING``.

3. **Stream Compaction (tcp_remove_urgbyte)**:
   When ``SO_OOBINLINE`` is disabled, ``tcp_remove_urgbyte()`` excises the
   urgent byte from the IOB chain:

   .. code-block:: text

      Before Removal (IOB chain with urgent byte 'U' at offset 2):
      +-----------------------------------------------------------+
      | IOB #1 (io_offset = 0, io_len = 5)                        |
      | Data: [ 'A' | 'B' | 'U' | 'C' | 'D' ]                     |
      +-----------------------------------------------------------+
                               |
                               | tcp_remove_urgbyte(head, offset=2)
                               | - Byte 'U' copied to conn->urg_data
                               | - memmove() shifts [ 'C' | 'D' ] left
                               | - io_len: 5 -> 4; io_pktlen: 5 -> 4
                               v
      After Removal (Compacted stream):
      +-----------------------------------------------------------+
      | IOB #1 (io_offset = 0, io_len = 4)                        |
      | Data: [ 'A' | 'B' | 'C' | 'D' | ' ' ]                     |
      +-----------------------------------------------------------+

   If an IOB becomes empty (``io_len == 0``), it is unlinked and returned to
   the free pool via ``iob_free()``.

4. **Sequence Number Advancement and SNDACK**:

   - **Pure Urgent Segments**: If the incoming segment contained only the
     urgent byte (reducing ``dev->d_len`` to 0 upon removal), ``tcp_urg_input()``
     advances the sequence number immediately via ``net_incr32(conn->rcvseq, 1)``.
   - **Mixed Segments**: If payload data remains after removal,
     ``*urg_removed = true`` is set. When the application or readahead logic
     consumes the remaining payload, advancing ``conn->rcvseq`` by ``dev->d_len``
     and zeroing ``dev->d_len``, ``tcp_input()`` verifies
     ``if (urg_removed && dev->d_len == 0)`` and increments ``conn->rcvseq`` by 1.
     The total sequence advancement matches the full original segment length.
   - **Forced ACK**: Whenever ``(flags & TCP_URGDATA) != 0``, ``tcp_input()``
     sets ``result |= TCP_SNDACK``, ensuring an acknowledgment is transmitted.

5. **Out-of-Order Processing (tcp_callback.c -> tcp_ofoseg_urg)**:
   If an out-of-order segment arrives with ``TCP_URG`` set, it is queued in
   ``conn->ofosegs``, and the stack enters ``TCP_URG_AWAITING``. When missing
   segments arrive and out-of-order buffers are committed, ``tcp_ofoseg_urg()``
   inspects each segment:

   .. code-block:: c

     if (conn->urg_state == TCP_URG_AWAITING &&
         TCP_SEQ_GTE(conn->urg_seq, seg->left) &&
         TCP_SEQ_LT(conn->urg_seq, seg->right))
       {
         uint32_t urg_offset = TCP_SEQ_SUB(conn->urg_seq, seg->left);

         iob_copyout(&conn->urg_data, seg->data, 1, urg_offset);
         conn->urg_state = TCP_URG_VALID;
         flags |= TCP_URGDATA;
         devif_conn_event(dev, TCP_URGDATA, conn->sconn.list);

         if (!_SO_GETOPT(conn->sconn.s_options, SO_OOBINLINE))
           {
             tcp_remove_urgbyte(&seg->data, urg_offset);
           }
       }

Socket and Polling APIs
=======================

Reading Urgent Data
-------------------

Applications read urgent data using ``recv()``, ``recvfrom()``, or ``recvmsg()``
with the ``MSG_OOB`` flag:

.. code-block:: c

  ssize_t ret = recv(sockfd, &oob_byte, 1, MSG_OOB);

Internally, kernel functions return negative error numbers (e.g., ``-EINVAL``).
At the user-space interface, functions return ``-1`` and set ``errno``.

.. list-table:: Return Codes for ``recv(..., MSG_OOB)``
   :header-rows: 1
   :widths: 25 75

   * - Return Value
     - Condition
   * - ``1``
     - Urgent byte copied to buffer. Transitions to ``TCP_URG_READ`` unless
       ``MSG_PEEK`` is set.
   * - ``0``
     - Caller passed ``len == 0``.
   * - ``-1 (EINVAL)``
     - Any of the following conditions:

       * Buffer pointer is ``NULL``.
       * ``SO_OOBINLINE`` is enabled on the socket.
       * State is ``TCP_URG_NOTSEEN`` (no urgent data arrived).
       * State is ``TCP_URG_READ`` (urgent byte was already read).
   * - ``-1 (EAGAIN / EWOULDBLOCK)``
     - The connection is in ``TCP_URG_AWAITING`` and the socket is
       non-blocking (``O_NONBLOCK`` or ``MSG_DONTWAIT``).
   * - ``-1 (EAGAIN / EWOULDBLOCK)``
     - A blocking read timed out waiting for urgent data (``SO_RCVTIMEO``).

Thread Synchronization and Cancellation Safety
----------------------------------------------

When a thread blocks on ``recv(..., MSG_OOB)`` waiting for announced urgent
data (``TCP_URG_AWAITING``):

1. The network lock is held on entry.
2. A cancellation handler is registered via ``tls_cleanup_push()``:

   .. code-block:: c

     info.tc_conn = conn;
     info.tc_cb   = &state.ir_cb;
     info.tc_sem  = &state.ir_sem;
     tls_cleanup_push(tls_get_info(), tcp_callback_cleanup, &info);

     ret = conn_dev_sem_timedwait(&state.ir_sem, true, timeout,
                                  &conn->sconn, conn->dev);
     tls_cleanup_pop(tls_get_info(), 0);

3. During ``conn_dev_sem_timedwait()``, the network lock is temporarily released
   so other tasks and network drivers can run, and re-acquired upon resume.
4. If the waiting thread is cancelled via ``pthread_cancel()`` while blocked,
   ``tcp_callback_cleanup()`` re-acquires the network lock, frees the callback
   structure, destroys the semaphore, and restores lock balance.

Polling via POLLPRI
-------------------

The arrival of urgent data can be monitored asynchronously:

* **Event Mask**: Set ``POLLPRI`` in ``struct pollfd.events``.
* **Trigger Condition**: In ``net/tcp/tcp_netpoll.c``, ``revents |= POLLPRI``
  is reported if either:

  1. A ``TCP_URGDATA`` event occurs on the connection; OR
  2. ``conn->urg_state == TCP_URG_VALID``.

Limitations
===========

The following POSIX / BSD features are not supported:

* **Signal Generation (SIGURG)**: Sockets do not generate asynchronous
  ``SIGURG`` signals upon urgent data arrival. Applications must monitor
  readiness via ``poll()`` or ``select()`` using ``POLLPRI``.
* **Socket Ownership (F_SETOWN / F_SETSIG)**: Socket owner assignment for
  signal delivery is not implemented.
* **Stream Mark Query (SIOCATMARK)**: The ``ioctl(sockfd, SIOCATMARK, &atmark)``
  call is not supported. Applications cannot query whether all pre-urgent stream
  bytes have been read.
* **RFC 793 Mode**: A runtime switch for RFC 793 mode (``tcp_stdurg=1``) is
  not available; BSD semantics are always used.

Testing and Verification
========================

No automated regression tests for TCP urgent data currently exist in
``apps/testing``. To validate interoperability, the following test scenarios
should be executed against a reference Linux host:

1. **Linux Interoperability**:

   - Verify communication with Linux ``sysctl net.ipv4.tcp_stdurg=0`` (standard BSD).
   - Confirm expected byte mismatch when tested against Linux ``sysctl net.ipv4.tcp_stdurg=1``.

2. **Segmentation and Loss**:

   - Out-of-order segment arrival: Verify ``tcp_ofoseg_urg()`` extracts urgent
     data correctly when packet ordering is shuffled.
   - Dropped segments: Verify retransmission preserves urgent pointer alignment.

3. **Socket Options and Flags**:

   - Toggle ``SO_OOBINLINE`` on and off during connection lifetime.
   - Verify ``MSG_PEEK`` retrieves the urgent byte without transitioning to
     ``TCP_URG_READ``.
   - Verify non-blocking sockets return ``EAGAIN`` during ``TCP_URG_AWAITING``.

Code Examples
=============

All examples use Allman bracing style and 2-space indentation.

Example 1: Transmitting an Out-of-Band Abort Command
----------------------------------------------------

.. code-block:: c

  #include <sys/socket.h>
  #include <stdio.h>
  #include <unistd.h>
  #include <errno.h>

  int send_oob_abort(int sockfd)
  {
    char cmd = 0x03; /* ASCII ETX / Ctrl+C */
    ssize_t ret;

    do
      {
        ret = send(sockfd, &cmd, 1, MSG_OOB);
      }
    while (ret < 0 && errno == EINTR);

    if (ret < 0)
      {
        perror("send MSG_OOB failed");
        return -1;
      }

    return 0;
  }

Example 2: Multiplexed Polling for Normal and Urgent Data
---------------------------------------------------------

.. code-block:: c

  #include <sys/socket.h>
  #include <poll.h>
  #include <stdio.h>
  #include <unistd.h>
  #include <errno.h>

  int wait_and_process(int sockfd)
  {
    struct pollfd pfd;
    int ret;

    pfd.fd     = sockfd;
    pfd.events = POLLIN | POLLPRI;

    do
      {
        ret = poll(&pfd, 1, 5000);
      }
    while (ret < 0 && errno == EINTR);

    if (ret <= 0)
      {
        return ret; /* Timeout (0) or error (< 0) */
      }

    /* Check for urgent out-of-band data first */

    if ((pfd.revents & POLLPRI) != 0)
      {
        char oob_byte;
        ssize_t n;

        do
          {
            n = recv(sockfd, &oob_byte, 1, MSG_OOB);
          }
        while (n < 0 && errno == EINTR);

        if (n == 1)
          {
            printf("Received urgent byte: 0x%02x\n", (unsigned char)oob_byte);
          }
      }

    /* Check for routine in-band stream data */

    if ((pfd.revents & POLLIN) != 0)
      {
        char buf[128];
        ssize_t n;

        do
          {
            n = recv(sockfd, buf, sizeof(buf) - 1, 0);
          }
        while (n < 0 && errno == EINTR);

        if (n > 0)
          {
            buf[n] = '\0';
            printf("Received normal data: %s\n", buf);
          }
      }

    return 0;
  }

Example 3: Enabling Inline Urgent Data (SO_OOBINLINE)
-----------------------------------------------------

.. code-block:: c

  #include <sys/socket.h>
  #include <stdio.h>

  int enable_inline_oob(int sockfd)
  {
    int opt = 1;
    int ret;

    ret = setsockopt(sockfd, SOL_SOCKET, SO_OOBINLINE,
                     &opt, sizeof(opt));
    if (ret < 0)
      {
        perror("setsockopt SO_OOBINLINE failed");
        return -1;
      }

    return 0;
  }
