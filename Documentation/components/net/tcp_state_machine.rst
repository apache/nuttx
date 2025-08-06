=============================
NuttX TCP State Machine Notes
=============================

This document describes how the current NuttX TCP stack implements TCP
state transitions. It is based on the in-tree implementation (primarily
in ``net/tcp``) and focuses on *what the code does today* rather than a
generic RFC 793 description.

Scope
=====

* TCP connection state is tracked per ``struct tcp_conn_s``.
* State transitions happen mainly in:

	* ``net/tcp/tcp_input.c`` (incoming segments and most transitions)
	* ``net/tcp/tcp_timer.c`` (timeouts and retransmissions)
	* ``net/tcp/tcp_conn.c`` (connect/listen-side allocation and initial state)
	* ``net/tcp/tcp_close.c`` (active close initiation)

State Representation
====================

NuttX stores TCP state in ``tcp_conn_s::tcpstateflags``.

* Bits 0-3 are the state (``TCP_STATE_MASK``).
* Bit 4 is a flag (``TCP_STOPPED``) used by the socket layer to stop data flow.

The state values are defined in ``include/nuttx/net/tcp.h``:

* ``TCP_CLOSED``
* ``TCP_ALLOCATED`` (NuttX-internal: allocated but not yet connected)
* ``TCP_SYN_RCVD``
* ``TCP_SYN_SENT``
* ``TCP_ESTABLISHED``
* ``TCP_FIN_WAIT_1``
* ``TCP_FIN_WAIT_2``
* ``TCP_CLOSE_WAIT``
* ``TCP_CLOSING``
* ``TCP_TIME_WAIT``
* ``TCP_LAST_ACK``
* ``TCP_STOPPED``

Supported vs Unsupported (RFC State View)
=========================================

NuttX largely follows the classic TCP state machine, the table below maps the traditional RFC 793 state names to what exists in
NuttX today.

.. list-table:: RFC TCP states and their NuttX support
	 :header-rows: 1
	 :widths: auto

	 * - RFC state name
		 - NuttX representation
		 - Supported
		 - Notes
	 * - CLOSED
		 - ``TCP_CLOSED``
		 - Yes
		 - Connection is unused/available.
	 * - LISTEN
		 - No ``tcpstateflags`` state
		 - Partially
		 - Listening is implemented via the listener table in ``net/tcp/tcp_listen.c``(``tcp_listenports[]``) rather than a per-connection LISTEN state.
	 * - SYN-SENT
		 - ``TCP_SYN_SENT``
		 - Yes
		 - Set by ``tcp_connect()`` in ``net/tcp/tcp_conn.c``.
	 * - SYN-RECEIVED
		 - ``TCP_SYN_RCVD``
		 - Yes
		 - Set when accepting an incoming SYN (new connection allocated for a listener).
	 * - ESTABLISHED
		 - ``TCP_ESTABLISHED``
		 - Yes
		 - Data transfer state.
	 * - FIN-WAIT-1
		 - ``TCP_FIN_WAIT_1``
		 - Yes
		 - Entered on active close (local FIN sent). However, it is currently unable to continue receiving data in this state
	 * - FIN-WAIT-2
		 - ``TCP_FIN_WAIT_2``
		 - Yes
		 - Entered after ACK for local FIN (when peer hasn't closed yet). However, it is currently unable to continue receiving data in this state
	 * - CLOSE-WAIT
		 - Not implemented
		 - Yes
		 - The TCP input path explicitly notes CLOSE_WAIT is not implemented; NuttX forces the application to close when FIN is received and moves directly toward ``TCP_LAST_ACK``.
	 * - CLOSING
		 - ``TCP_CLOSING``
		 - Yes
		 - Used for simultaneous close handling.
	 * - LAST-ACK
		 - ``TCP_LAST_ACK``
		 - Yes
		 - Used after receiving FIN and sending FIN in response.
	 * - TIME-WAIT
		 - ``TCP_TIME_WAIT``
		 - Yes
		 - Used after the close handshake; timer-driven cleanup.

Note on ``TCP_ALLOCATED``
-------------------------

``TCP_ALLOCATED`` is NuttX-specific and has no direct RFC state name.
It is the pre-connect/pre-accept state for a newly created socket connection.

High-level Transition Summary
=============================

This section summarizes the most common state paths.

Active open (connect)
---------------------

Typical client-side flow:

::

	TCP_ALLOCATED
		-> TCP_SYN_SENT        (tcp_connect() prepares SYN)
		-> TCP_ESTABLISHED     (tcp_input receives SYN|ACK and replies ACK)

Passive open (listen/accept)
----------------------------

Listening sockets are registered in the listener table (not a LISTEN state).
When a SYN arrives:

::

	listener in tcp_listenports[]
		-> new conn: TCP_SYN_RCVD  (tcp_allocaccept() in tcp_conn.c)
		-> TCP_ESTABLISHED         (tcp_input receives final ACK)
		-> accept() wakes up       (tcp_accept_connection())

Graceful close (active close)
-----------------------------

When the application initiates a close (or ``shutdown(SHUT_WR)``), the stack
sends FIN and transitions:

::

	TCP_ESTABLISHED
		-> TCP_FIN_WAIT_1
		-> TCP_FIN_WAIT_2          (ACK of our FIN)
		-> TCP_TIME_WAIT           (FIN from peer)
		-> TCP_CLOSED              (timer expiry)

Simultaneous close
------------------

If FIN is received while we are in ``TCP_FIN_WAIT_1`` and our FIN has not been
fully ACKed, NuttX can enter ``TCP_CLOSING``:

::

	TCP_FIN_WAIT_1
		-> TCP_CLOSING
		-> TCP_TIME_WAIT           (ACK of our FIN)

Passive close (peer closes first)
---------------------------------

When FIN is received in ESTABLISHED, the application is notified
via callbacks. the stack sends ACK and goes to ``TCP_CLOSE_WAIT``:

::

	TCP_ESTABLISHED
		-> TCP_CLOSE_WAIT          (FIN received)
		-> TCP_CLOSED              (ACK of our FIN)

Detailed State Handling
=======================

TCP_SYN_SENT
------------

* Entered by ``tcp_connect()`` (``net/tcp/tcp_conn.c``).
* On receiving ``SYN|ACK`` with a valid ACK:

	* Parses options (e.g., MSS).
	* Sets ``TCP_ESTABLISHED``.
	* Updates ``rcvseq`` and window tracking.
	* Notifies the socket layer using ``TCP_CONNECTED``.

* On unexpected control segments or failure:

	* The connection is aborted (``TCP_ABORT`` callback) and a RST may be sent.

TCP_SYN_RCVD
------------

* Entered for a newly accepted connection when a SYN matches a listener.
	Allocation and initialization occur in ``tcp_allocaccept()``
	(``net/tcp/tcp_conn.c``).
* A SYN-ACK is sent. The retransmission is handled by ``tcp_timer.c``.
* On receiving the final ACK (``TCP_ACKDATA``):

	* Transition to ``TCP_ESTABLISHED``.
	* ``tcp_accept_connection()`` is called to hand the connection to the
		listening socket/accept logic.

TCP_ESTABLISHED
---------------

* Normal data transfer occurs here.
* Incoming data and ACK processing is handled in ``net/tcp/tcp_input.c``.
* If a FIN is received:

	* The application is notified (``TCP_CLOSE`` flag is included in callback).
	* NuttX transitions to ``TCP_CLOSE_WAIT`` and sends ``ACK``.

TCP_CLOSE_WAIT
--------------

* Only entered when a FIN is received in ESTABLISHED.
* The application is notified (``TCP_CLOSE`` flag in callback).
* NuttX can send data until the application initiates close.
* On application close request:
	* NuttX sends FIN and transitions to ``TCP_LAST_ACK``.

TCP_FIN_WAIT_1
--------------

* Entered when the application requests a graceful close.
	This is initiated in ``net/tcp/tcp_appsend.c`` when the callback result
	contains ``TCP_CLOSE``.

* On receiving FIN:

	* If the FIN also ACKs our FIN and ``tx_unacked == 0``: transition to
		``TCP_TIME_WAIT``.
	* Otherwise: transition to ``TCP_CLOSING``.
	* In both cases, ACK the peer FIN.

* On receiving an ACK that completes ACK of our FIN (and no FIN from peer):

	* Transition to ``TCP_FIN_WAIT_2``.

* Data received in FIN_WAIT_1:

	* Current behavior is to send a RST and force ``TCP_CLOSED``.
	* The implementation notes this as a TODO to improve shutdown behavior.

TCP_FIN_WAIT_2
--------------

* Waiting for the peer FIN after our FIN was ACKed.
* On receiving FIN:

	* Transition to ``TCP_TIME_WAIT``.
	* ACK the FIN and notify close.

* Data received in FIN_WAIT_2:

	* Current behavior is to send a RST and force ``TCP_CLOSED``.

TCP_CLOSING
-----------

* Simultaneous close case.
* When the ACK for our FIN is received (``TCP_ACKDATA``):

	* Transition to ``TCP_TIME_WAIT``.

TCP_LAST_ACK
------------

* Entered after FIN is received in ESTABLISHED and the application chooses
	to close, causing the stack to send FIN.
* On receiving ACK for our FIN (``TCP_ACKDATA``):

	* Transition to ``TCP_CLOSED``.
	* Notify close via callback.

TCP_TIME_WAIT
-------------

* NuttX responds to segments by sending an ACK.
* Cleanup is timer-driven (see ``tcp_timer.c``):

	* ``TCP_TIME_WAIT`` are handled as "wait for timeout" states.
	* When the per-connection timer expires, the state becomes ``TCP_CLOSED``.

Timers, Retransmissions, and Failure Handling
=============================================

The TCP timer handler in ``net/tcp/tcp_timer.c`` drives:

* Retransmission for connections with ``tx_unacked > 0``.
* State-specific retransmit behavior:

	* ``TCP_SYN_RCVD``: retransmit SYN-ACK.
	* ``TCP_SYN_SENT``: retransmit SYN.
	* ``TCP_ESTABLISHED``: request retransmit via callback (``TCP_REXMIT``).
	* ``TCP_FIN_WAIT_1``, ``TCP_CLOSING``, ``TCP_LAST_ACK``: retransmit FIN|ACK.

* Timeout cleanup:

	* ``TCP_SYN_RCVD``: if SYN-ACK retransmits exceed limit, the half-open
		connection is closed and freed.
	* ``TCP_SYN_SENT`` and established cases: if retransmits exceed limit, the
		connection is closed, the socket is notified (``TCP_TIMEDOUT``), and a
		RST may be sent.

Deviations and Notable Simplifications
======================================

* LISTEN is not an explicit TCP state; it is represented by listener table entries.
* FIN_WAIT_* data handling is currently strict: received payload data in
	FIN_WAIT_1/2 results in sending RST and closing the connection.
* RST processing is intentionally simple (accept RST and close).

Where to Look in the Code
=========================

* State definitions: ``include/nuttx/net/tcp.h``
* Incoming-segment state logic: ``net/tcp/tcp_input.c``
* Retransmission/timeout logic: ``net/tcp/tcp_timer.c``
* Connect path / SYN_SENT setup: ``net/tcp/tcp_conn.c``
* Accept path / SYN_RCVD allocation: ``net/tcp/tcp_conn.c``
* Active close initiation: ``net/tcp/tcp_close.c`` and ``net/tcp/tcp_shutdown.c``
* Listener table (LISTEN semantics): ``net/tcp/tcp_listen.c``
