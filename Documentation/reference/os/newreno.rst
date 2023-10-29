==========================
Congestion Control NewReno
==========================

NewReno congestion control algorithm is used to solve the problem of network congestion breakdown, which includes:
 - Slow Start
 - Congestion Avoidance
 - Fast Retransmission
 - Fast Recovery.

 The implementation refers to RFC6582 and RFC5681. In addition, we optimize the congestion algorithm. In the congestion avoidance state, the maximum congestion window (max_cwnd) is used to limit the excessive growth of cwnd and prevent network jitter caused by congestion. Maximum congestion window (max_cwnd) is updated with the current congestion window (cwnd) and the update weight is 0.875 when an RTO timeout occurs.

Workflow
========

The NewReno on the tcp sender adjusts the cwnd and ssthresh based on received ack and Retransmitted Timeout (RTO) events.

Using the cwnd, together with snd_wnd, controls the number of bytes sent to the network. Here's how newreno works, as following:

- Initialize the ssthresh and cwnd, on establishing the tcp connection.
- When the ack is received, check whether the ack is repeated.

 + If yes, increase the dupack counts. If the dupack exceeds the Fast Retransmission Threshold 3, after retransmitting the lost segments (Fast Retransmission), enter to  the Fast Recovery state.
 + If no, receive the new ack.

   * If the current ackno is bigger than fr_ack which is the snd_seq when Fast Retransmission ocurrs, exit the Fast Recovery state and enter to congestion avoidance.
   * If the cwnd is less than ssthresh, increase the cwnd on slow start state.
   * If the cwnd is greater than or equal to ssthresh, the increased cwnd can not exceed max_cwnd.

- when RTO times out, reset the values of cwnd and ssthresh, update the max_cwnd, and enter to Slow Start state.
- When sending a segment, the minimum value of cwnd and snd_wnd is used to calculate the number of bytes that can be sent.

The simple state transition diagram of the NewReno is shown below.

::

                                    |           ^
                                    | ------------------------
                                    | initialize cwnd ssthresh
                                    V
                              +------------+
             .--------------->| Slow Start |-----------------.
             |                +------------+                 |
             |                     |  |                      |
             |    timeout          |  |  recv dup ack        | recv new ack
             |------------------   |  |  ---------------     | ----------------
             |reset cwnd ssthresh  |  |  dupack >= 3         | cwnd >= ssthresh
             |update max_cwnd      |  |  fr_ack = snd_seq    |
             |<--------------------'  |<------------------.  |
             |                        |                   |  |
             |                        v                   |  V
             |                    +--------+     +--------------------+
             |                    |   FT   |     |Congestion Avoidance|
             |                    +--------+     +--------------------+
             |                        |                   ^  |
             |              retransmit|lost segment       |  |
             |                        |                   |  |
             |                        |      recv new ack |  |
             |                        v      ------------ |  |
             |                    +--------+ ack > fr_ack |  |
             |                    |   FR   |--------------'  |
             |                    +--------+                 |
             |                        |                      |
             |                        v                      v
             '-----------------------------------------------'

Configuration Options
=====================
``NET_TCP_CC_NEWRENO``
  Enable or disable NewRenofunction.

  Depends on ``NET_TCP_FAST_RETRANSMIT``.

Test
====


Test topology
-------------

::

                         IP:10.0.1.1

                         +--------+
                 --------| nuttx0 |--------
                 |       +--------+       |
                 |          /|\           |
                 |           |            |
                 |       +-------+        |
                 |       | ifb0  |        |
                 |       +-------+        |
                \|/         /|\          \|/
             +-------+       |        +-------+
             | tap0  |------/ \-------| tap1  |
             +-------+                +-------+
                /|\                      /|\
                 |                        |
                \|/                      \|/
             +-------+                +-------+
        sim1 | eth0  |                | eth0  | sim2
             +-------+                +-------+

             IP:10.0.1.3              IP:10.0.1.4

Test steps
----------

Test the function on the Ubuntu 22.04 x86_64 with NuttX SIM by following steps:

:1.Configure the test environment:

- Set the nuttx0 inbound speed to 10Mbps.

 ..  code-block:: bash

    # Load fib module, and start ifb0 interface
    modprobe ifb
    ip link set dev ifb0 up

    # Import the nuttx0 ingress packets into ifb0
    tc qdisc add dev nuttx0 handle ffff: ingress
    tc filter add dev nuttx0 parent ffff: u32 match u32 0 0 action mirred egress redirect dev ifb0

    # Limit nuttx0 ingress 10Mbps
    tc qdisc add dev ifb0 root tbf rate 10Mbit latency 50ms burst 1540

- configure the sim simulator.

 + Start iperf3 server on ubuntu.

 ..  code-block:: bash

     iperf3 -s -i1 -p10003  #for sim1
     iperf3 -s -i1 -p10004  #for sim2


 + start the emulators sim1 and sim2 and configure ip addresses.

 ..  code-block:: bash

  # start and configure sim1
  start gdb nuttx
  ifconfig eth0 10.0.1.3

  # start and configure sim2
  start gdb nuttx
  ifconfig eth0 10.0.1.4 # sim2


:2.Stream Testing:


- Use iperf3 to perform the stream testing.

 ..  code-block:: bash

  iperf3 -c 10.0.1.1 -i1 -t60 -p10003 # sim1

  iperf3 -c 10.0.1.1 -i1 -t60 -p10004 # sim2


:3.Comparison Testing:

 Compares the test results of enabling and disabling NewReno.


Test results
------------

 The test results should indicate that the total network throughput was significantly increased when NewReno congestion control was enabled, which was close to the actual total network bandwidth, and the rates of both sim devices were stable.
