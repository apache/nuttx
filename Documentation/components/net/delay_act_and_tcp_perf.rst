===============================
Delayed ACK and TCP Performance
===============================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Delayed+ACK+and+TCP+Performance

uIP and NuttX
=============

The heart of the NuttX IP stack derived from Adam Dunkel's tiny `uIP 
stack <http://sourceforge.net/projects/uip-stack/>`_ back at version 1.0.
The NuttX TCP/IP stack contains the uIP TCP state machine and 
some uIP "ways of doing things," but otherwise, there is 
now little in common between these two designs.

**NOTE**: uIP is also built into Adam Dunkel's 
`Contiki <http://contiki.sourceforge.net/docs/2.6/a01793.html>`_ 
operating system.

uIP, Delayed ACKs, and Split Packets
====================================

In uIP, TCP packets are sent and ACK'ed one at a time.
That is, after one TCP packet is sent, the next packet cannot 
be sent until the previous packet has been ACKed by the 
receiving side.
The TCP protocol, of course, supports sending multiple packets 
which can be ACKed be the receiving time asynchronously.
This one-packet-at-a-time logic is a simplification in the 
uIP design; because of this, uIP needs only a single packet 
buffer any you can use uIP in even the tiniest environments.
This is a good thing for the objectives of uIP.

Improvements in packet buffering is the essential improvement 
that you get if upgrade from Adam Dunkel's uIP to his 
`lwIP <http://savannah.nongnu.org/projects/lwip/>`_ stack.
The price that you pay is in memory usage.

This one-at-a-time packet transfer does create a performance 
problem for uIP:
RFC 1122 states that a host may delay ACKing a packet for up 
to 500ms but must respond with an ACK to every second segment.
In the baseline uIP, the effectively adds a one half second 
delay between the transfer of every packet to a recipient 
that employs this delayed ACK policy!

uIP has an option to work around this:
It has logic that can be enable to split each packet into half, 
sending half as much data in each packet.
Sending more, smaller packets does not sound like a performance 
improvement.
This tricks the recipient that follows RFC 1122 into receiving 
the two, smaller back-to-back packets and ACKing the second 
immediately.
References: `uip-split.c <http://contiki.sourceforge.net/docs/2.6/a00427_source.html>`_ 
and `uip-split.h <http://contiki.sourceforge.net/docs/2.6/a00428.html>`_.

The NuttX TCP/IP Stack and Delay ACKs
=====================================

The NuttX, low-level TCP/IP stack does not have the 
limitations of the uIP TCP/IP stack.
It can send numerous TCP/IP packets regardless of 
whether they have been ACKed or not.
That is because in NuttX, the accounting for which 
packets have been ACKed and which have not has been 
moved to a higher level in the architecture.

NuttX includes a standard, BSD socket interface on top 
of the low-level TCP/IP stack.
It is in this higer-level, socket layer where the ACK 
accounting is done, specifically in the function 
`send() <http://pubs.opengroup.org/onlinepubs/009695399/functions/send.html>`_.
If you send a large, multi-packet buffer via ``send()``, 
it will be broken up into individual packets and each 
packet will be sent as quickly as possible, with no 
concern for whether the previous packet has been ACKed or not.

However, the NuttX ``send()`` function will not return to 
the caller until the final packet has been ACKed.
It does this to assure that the callers data was sent 
successfully (or not).
This behavior means that if an odd number of packets 
were sent, there could still be a delay after the final 
packet before ``send()`` receives the ACK and returns.

So the NuttX approach is similar to the uIP way of doing 
things, but does add one more buffer, the user provided 
buffer to ``send()``, that can be used to improve TCP/IP 
performance (of course, this user provided buffer is 
also required by in order to be compliant with ``send()``"" 
`specification <http://pubs.opengroup.org/onlinepubs/009695399/functions/send.html>`_.

The NuttX Split Packet Configuration
====================================

But what happens if all of the user buffer is smaller than 
the MSS of one TCP packet?
Suppose the MTU is 1500 and the user I/O buffer is only 
512 bytes?
In this case, ``send()`` performance degenerates to the same 
behavior as uIP:
An ACK is required for each packet before ``send()`` can 
return and before ``send()`` can be called again to send 
the next packet.

And the fix? A fix has recently been contributed by 
Yan T that works in a similar way to uIP split 
packet logic:
In ``send()``, the logic normally tries to send a full packet 
of data each time it has the opportunity to do so.
However, if the configuration option ``CONFIG_NET_TCP_SPLIT=y`` 
is defined, the behavior of ``send()`` will change in the 
following way:

* ``send()`` will keep track of `even` and `odd` packets; `even` 
  packets being those that we do not expect to be ACKed 
  and `odd` packets being the those that we do expect to 
  be ACKed.
* ``send()`` will then reduce the size of even packets as 
  necessary to assure that an even number of packets is 
  always sent. Every call to send will result in an even 
  number of packets being sent.

This clever solution tricks the RFC 1122 recipient in the 
same way that uIP split logic does.
So if you are working with hosts the following the RFC 1122 
ACKing behavior and you have MSS sizes that are larger that 
the average size of the user buffers, then your throughput 
can probably be greatly improved by enabling ``CONFIG_NET_TCP_SPLIT=y``

NOTE: NuttX is `not` an RFC 1122 recipient; NuttX will ACK 
every TCP/IP packet that it receives.

Write Buffering
===============

The best technical solution to the delayed ACK problem 
would be to support `write buffering`.
Write buffering is enabled with ``CONFIG_NET_TCP_WRITE_BUFFERS``. 
If this option is selected, the NuttX networking layer will 
pre-allocate several write buffers at system initialization 
time. The sending a buffer of data then works like this:

* ``send()`` (1) obtains a pre-allocated write buffer from a free 
  list, and then (2) simply copies the buffer of data that the 
  user wishes to send into the allocated write buffer. If no 
  write buffer is available, ``send()`` would have to block waiting 
  for free write buffer space.
* ``send()`` then (3) adds the write buffer to a queue of outgoing 
  data for a TCP socket. Each open TCP socket has to support 
  such a queue. ``send()`` could then (4) return success to the 
  caller (even thought the transfer could still fail later).
* Logic outside of the ``send()`` implementation manages the actual 
  transfer of data from the write buffer. When the Ethernet 
  driver is able to send a packet on the TCP connection, this 
  external logic (5) copies a packet of data from the write 
  buffer so that the Ethernet driver can perform the 
  transmission (a `zero-copy` implementation would be preferable). 
  Note that the data has to remain in the write buffer for now; 
  it may need to be re-transmitted.
* This external logic would also manage the receipt TCP ACKs. 
  When TCP peer acknowledges the receipt of data, the 
  acknowledged portion of the data can the (6) finally 
  be deleted from the write buffer.

The following options configure TCP write buffering:

* ``CONFIG_NET_TCP_WRITE_BUFSIZE``: The size of one TCP write buffer.
* ``CONFIG_NET_NTCP_WRITE_BUFFERS``: The number of TCP 
  write buffers (may be zero to disable TCP/IP write buffering)

NuttX also supports TCP read-ahead buffering. This option 
is enabled with ``CONFIG_NET_TCP_READAHEAD``. TCP read-ahead 
buffer is necessary on TCP connections; otherwise data 
received while there is no ``recv()`` in place would be lost. 
For consistency, it would be best if such a TCP write 
buffer implementation worked in a manner similar to the 
existing TCP read-ahead buffering.

The following lists the NuttX configuration options 
available to configure the TCP read-ahead buffering feature:

* ``CONFIG_NET_TCP_READAHEAD_BUFSIZE``: The size of one TCP read-ahead buffer.
* ``CONFIG_NET_NTCP_READAHEAD_BUFFERS``: The number of TCP 
  read-ahead buffers (may be zero to disable TCP/IP read-ahead buffering)

A future enhancement is to combine the TCP write buffer 
management logic and the TCP read-ahead buffer management 
so that one common pool of buffers can be used for both 
functions (this would probably also require additional logic 
to `throttle` read-buffering so that received messages do not 
consume all of the buffers).