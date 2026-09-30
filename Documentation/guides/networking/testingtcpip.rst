=============================
Testing TCP/IP Network Stacks
=============================

When working on the network stack there is a need to test and verify the changes
made. While problems may be discovered by chance, it is hard to reproduce such
situations. The following sections show some methods to stress the target or
generate some specific traffic.

In the examples the target has the IP address 192.168.2.135

SYN Flood Attack
================

Flood the target with SYN packets to exhaust its resources.
It's a good way to test the network driver's buffer management.

.. code-block:: bash

   sudo hping3 --flood -S -p 80 192.168.2.135

Building Packets with Scapy
===========================

A wonderful network testing tool is the Scapy lib.
It enables you to build pretty much any packet constellation you need for testing.

You have to add an iptables rule to prevent outgoing RST packets from the OS's
networking stack which does nothing know about our test connection.

To disable outgoing RST packets:

.. code-block:: bash

   sudo iptables -A OUTPUT -p tcp --tcp-flags RST RST -d 192.168.2.135 -j DROP

For removing the rule:

.. code-block:: bash

   sudo iptables -D OUTPUT -p tcp --tcp-flags RST RST -d 192.168.2.135 -j DROP

**Testing Re-transmission behavior**

When sending a 3-way handshake only, the target should time out and reset the
connection.

The following Python Scapy script starts a HTTP request without further responding.
The stack should start re-transmit the packets and finally time out.

.. code-block:: python

   #!/usr/bin/env python
 
   import logging
   logging.getLogger("scapy.runtime").setLevel(logging.ERROR)
   from scapy.all import *
 
   get = 'GET / HTTP/1.1\r\n\r\n'
 
   ip = IP(dst="192.168.2.135")
   port = RandNum(1024, 65535)
 
   # Create SYN packet
   SYN = ip/TCP(sport=port, dport=80, flags="S", seq=42)
 
   # Send SYN and receive SYN,ACK
   SYNACK = sr1(SYN)
 
   # Create ACK with GET request
   ACK = ip/TCP(sport=SYNACK.dport, dport=80, flags="A", seq=SYNACK.ack, ack=SYNACK.seq + 1)
 
   # SEND our ACK
   send(ACK)
 
   reply, err = sr(ip/TCP(sport=SYNACK.dport, dport=80, flags="A", seq=SYNACK.ack, ack=SYNACK.seq + 1) / get)

Simulating Packet Loss
======================

With simulating packet loss one can test the re-transmission behavior of the
target stack.

To start packet loss:

.. code-block:: bash

   # for randomly dropping 10% of incoming packets:
   sudo iptables -A INPUT -m statistic --mode random --probability 0.1 -j DROP
 
   # and for dropping 10% of outgoing packets:
   sudo iptables -A OUTPUT -m statistic --mode random --probability 0.1 -j DROP

To remove the rules:

.. code-block:: bash

   # for the incoming packets:
   sudo iptables -D INPUT -m statistic --mode random --probability 0.1 -j DROP
 
   # and for the outgoing packets
   sudo iptables -D OUTPUT -m statistic --mode random --probability 0.1 -j DROP

Fuzz-Testing
============

For fuzz testing network applications the excellent
`SPIKE <https://www.immunitysec.com/resources-freesoftware.shtml>`_ tool can be
used. To make it compile under Ubuntu 14.04 LTS you have to add
``-fno-stack-protector`` to CFLAGS.

SPIKE complained about missing SSL libs. I simply linked the existing to the needed
filenames::

  /lib/i386-linux-gnu$ sudo ln -s ./libssl.so.1.0.0 ./libssl.so.0
  /lib/i386-linux-gnu$ sudo ln -s ./libcrypto.so.1.0.0 ./libcrypto.so.0

Maybe those lib versions aren't the expected by SPIKE but if you don't use SSL
it works fine.

SPIKE provides a proxy server to record requests to your web application. Based
on these requests a application specific fuzz test can be generated.

.. code-block:: bash

   ~/SPIKE/src$ mkdir requests && cd requests
   # Record requests to the target at 192.168.2.135 on port 80
   ~/SPIKE/src/requests$ ../webmitm -t 192.168.2.135 -p 80

Now use your web application through localhost to record some requests. Then you
can generate your application-specific fuzz test from the recorded requests.

.. code-block:: bash

   ~/SPIKE/src$ ./makewebfuzz.pl ./requests/http_request-1.0 > myfuzz.c
   ~/SPIKE/src$ gcc ./myfuzz.c -I../include -o myfuzz -L. -ldlrpc -ldl

Now you can fuzz your target:

.. code-block:: bash

   ~/SPIKE/src$ LD_LIBRARY_PATH=. ./myfuzz 192.168.2.135 80
