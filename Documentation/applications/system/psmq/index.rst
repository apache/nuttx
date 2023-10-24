========================================
``psmq`` Publish Subscribe Message Queue
========================================

``psmq`` is publish subscribe message queue. It's a set of programs and libraries
to implement publish/subscribe way of inter-process communication on top of
POSIX message queue.

Manuals, source code and more info at: https://psmq.bofc.pl

Little demo using ``psmqd`` broker, ``psmq_pub`` and ``psmq_sub``:

Start broker and make it log to file::

  nsh> psmqd -b/brok -p/sd/psmqd/psmqd.log &

Start subscribe thread that will read all messages send on ``/can/*`` and
``/adc/*`` topic, and dump all readings to file::

  nsh> psmq_sub -n/sub -b/brok -t/can/* -t/adc/* -o/sd/psmq-sub/can.log &
  n/connected to broker /brok
  n/subscribed to: /can/*
  n/subscribed to: /adc/*
  n/start receiving data
  n/reply timeout set 100

Publish some messages::

  nsh> psmq_pub -b/brok -t/can/engine/rpm -m50
  nsh> psmq_pub -b/brok -t/adc/volt -m30
  nsh> psmq_pub -b/brok -t/can/room/10/temp -m23
  nsh> psmq_pub -b/brok -t/pwm/fan1/speed -m300

Check out subscribe thread logs::

  nsh> cat /sd/psmq-sub/can.log

  [2021-05-23 17:53:59] p:0 l:   3  /can/engine/rpm  50
  [2021-05-23 17:53:59] p:0 l:   3  /adc/volt  30
  [2021-05-23 17:53:59] p:0 l:   3  /can/room/10/temp  23

As you can see ``/pwm/fan1/speed`` hasn't been received by subscribe thread,
since we didn't subscribe to it.

Content:

- ``psmqd`` – broker, relays messages between clients.
- ``psmq_sub`` – listens to specified topics, can be used as logger for
  communication (optional).
- ``psmq_pub`` – publishes messages directly from shell. Can send binary data, but
  requires pipes, so on nuttx it can only send ASCII.
- ``libpsmq`` – library used to communicate with the broker and send/receive
  messages.
