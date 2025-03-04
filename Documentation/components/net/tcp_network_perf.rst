=======================
TCP Network Performance
=======================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/TCP+Network+Performance


(Abstracted and extended from a discussion from the NuttX Google group)

Question
========

For some unknown reason, I am seeing poor TCP network performance.

Answer
======

First let's talk about TCP send performance.

Source of Performance Bottlenecks
---------------------------------

General TCP send performance is not determined by the TCP stack as much 
as it is by the network device driver. Bad network performance is due 
to time lost `BETWEEN` packet transfers. The packet transfers themselves 
go at the wire speed*. So if you want to improve performance on a 
given network, you have to reduce time lost between transfers. 
There is no other way.

Ignoring Ethernet issues like collisions, back-off delays, 
inter-packet gaps (IPG), etc.

The time between packets is limited primarily by the buffering 
design of the network driver. If you want to improve performance, 
then you must improve the buffering at the network driver. 
You need to support many full size (1500 byte) packet buffers. 
You must be able to query the network for new data to transfer, 
and queue those transfers in packet buffers. In order to reach 
peak performance, the network driver must have the next transfer 
buffered and ready-to-go before the previous transfer is finished 
to minimize the GAP between packet transfers.

Different network devices also support more or less efficient 
interfaces: The worst performing support interfaces that can 
handle only one packet at a time, the best performing are able 
to retain linked lists of packet buffers in memory and perform 
scatter-gather DMA for a sequence of packets.

In the NuttX TCP stack, you can also improve performance by 
enabling TCP write buffering. But the driver is the real key.

It would be good to have a real in-depth analysis of the 
network stack performance to identify bottlenecks and 
generate ideas for performance improvement. No one has 
ever done that. If I were aware of any stack related 
performance issue, I would certainly address it.

RFC 1122
--------

There is one important feature missing the NuttX TCP that 
can help when there is no write buffering: Without write 
buffering send() will not return until the transfer has 
been ACKed by the recipient. But under RFC 1122, the host 
need not ACK each packet immediately; the host may wait 
for 500 MS before ACKing. This combination can cause very 
slow performance when small, non-buffered transfers are 
made to an RFC 1122 client. However, the RFC 1122 must 
ACK at least every second (odd) packet so sequences of 
packets with write buffering enabled do not suffer from 
this problem.

`Update:  RFC 1122 support was added to the NuttX TCP 
stack with commit 66ef6d143a627738ad7f3ce1c065f9b1f3f303b0 
in December of 2019.  That, however, that affects only 
received packet ACK behavior and has no impact on transmitted 
packet performance; write buffering is still recommended.`

TCPBlaster
----------

I created a new test application at ``apps/examples/tcpblaster`` to 
measure TCP performance and collected some data for the 
configuration that happens to be on my desk. The `tcpblaster` 
test gives you the read and write transfer rates in ``Kb/sec`` 
(I won't mention the numbers because I don't believe they 
would translate any other setup and, hence, would be 
misleading).

There is a nifty `TCP Throughput Tool <https://www.switch.ch/network/tools/tcp_throughput/>`_ 
that gives some theoretical upper limits on performance. 
The tool needs to know the ``MSS`` (which is the Ethernet 
packet size that you configured minus the size of the 
Ethernet header, 14), the round-trip time (``RTT``)in 
milliseconds (which you can 
get from the Linux host ping), and a loss constant (which 
I left at the default). With these values, I can determine 
that the throughput for the NuttX TCP stack is approximately 
at the theoretical limits. You should not be able to do 
better any better than that (actually, it performs above 
the theoretical limit, but I suppose that is why it is 
"theoretical").

So, If you are unhappy with your network performance, the I 
suggest you run the `tcpblaster` test, use that data 
(along with the ``RTT`` from ping) with the 
`TCP Throughput Tool <https://www.switch.ch/network/tools/tcp_throughput/>`_. 
If you are still unhappy with the performance, don't go 
immediately pointing fingers at the stack (which everyone does). 
Instead, you should focus on optimizing your network 
configuration settings and reviewing the buffer handling 
of the Ethernet driver in you MCU.

If you do discover any significant performance issues 
with the stack I will of course gladly help you resolve 
them. Or if you have ideas for improved performance, 
I would also be happy to hear those.

What about Receive Performance?
-------------------------------

All of the above discussion concerns `transmit performance`, 
i.e., "How fast can we send data over the network?" The other 
side is receive performance. Receive performance is very 
different thing. In this case it is the remote peer who is 
in complete control of the rate at which packets appear on 
the network and, hence, responsible for all of the raw bit 
transfer rates.

However, we might also redefine performance as the number of 
bytes that were `successfully` transferred. In order for the 
bytes to be successfully transferred they must be successfully 
received and processed on the NuttX target. If we fail in 
this if the packet is `lost` or `dropped`. A packet is lost if 
the network driver is not prepared to receive the packet when 
it was sent. A packet is dropped by the network if it is 
received but could not be processed either because there 
is some logical issue with the packet (not the case here) 
or if we have no space to buffer the newly received packet.

If a TCP packet is lost or dropped, then the penalty is big: 
The packet will not be ACKed, the remote peer may send a 
few more out-of-sequence packets which will also be dropped. 
Eventually, the remote peer will time out and retransmit 
the data from the point of the lost packet.

There is logic in the TCP protocol to help manage these data 
overruns. The TCP header includes a TCP `receive window` which 
tells the remote peer how much data the receiver is able to 
buffer. This value is sent in the ACK to each received 
packet. If well tuned, this receive window could possibly 
prevent packets from being lost due to the lack of 
read-ahead storage. This is a little better. The remote 
peer will hold off sending data instead of timing out and 
re-transmitting. But this is still a loss of performance; 
the gap between the transfer of packets caused by the hold-off 
will result in a reduced transfer rate.

So the issues for good reception are buffering and processing 
time. Buffering again applies to handling within the driver 
but unlike the transmit performance, this is not typically 
the bottleneck. And there is also a NuttX configuration 
option that controls `read-ahead` buffering of TCP packets. 
The buffering in the driver must be optimized to avoid lost 
packets; the ` buffering can be tuned to minimize 
the number packets dropped because we have no space to buffer them.

But the key to receive perform is management of processing 
delays. Small processing delays can occur in the network 
driver or in the TCP stack. But the major source of 
processing delay is the application which is the ultimate 
consumer of the incoming data. Imagine, for example, 
and FTP application that is receiving a file over a 
TCP and writing the file into FLASH memory. The primary 
bottleneck here will be the write to FLASH memory which 
is out of the control of software.

We obtain optimal receive performance when the processing 
delays keep up with the rate of the incoming packets. 
If the processing data rate is even slightly slower 
then the receive data rate, then there will be a 
growing `backlog` of buffered, incoming data to be 
processed. If this backlog continues to grow then 
eventually our ability to buffer data will be exhausted, 
packets will be held off or dropped, and performance 
will deteriorate. In an environment where a high-end, 
remote peer is interacting with the low-end, embedded 
system, that remote peer can easily overrun the 
embedded system due to the embedded system's limited 
buffering space, its much lower processing capability, 
and its slower storage peripherals.