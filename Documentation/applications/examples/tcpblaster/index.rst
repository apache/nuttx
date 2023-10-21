``tcpblaster`` TCP Performance Test
===================================

The ``tcpblaster`` example derives from the ``nettest`` example and basically
duplicates that example when the ``nettest`` PERFORMANCE option is selected.
``tcpblaster`` has a little better reporting of performance stats, however.

To set up, do ``make menuconfig`` and select the Apps → Examples → tcpblaster.
By default, nuttx will the be the client which sends data; and the
host computer (Linux, macOS, or Windows) will be the server.

Set up networking so the nuttx computer can ping the host, and the host can ping
nuttx. Now you are ready to run the test.

On host::

  $ ./tcpserver
  Binding to IPv4 Address: 00000000
  server: Accepting connections on port 5471

On nuttx::

  nsh> tcpclient
  Connecting to IPv4 Address: 0100000a
  client: Connected
  [2014-07-31 00:16:15.000] 0: Sent 200 4096-byte buffers:    800.0 KB (avg   4.0 KB) in   0.18 seconds (4444.4 KB/second)

Now on the host you should see something like::

  $ ./tcpserver
  Binding to IPv4 Address: 00000000
  server: Accepting connections on port 5471
  server: Connection accepted -- receiving
  [2020-02-22 16:17:07.000] 0: Received 200 buffers:   502.9 KB (buffer average size:   2.5 KB) in   0.12 seconds (4194.8 KB/second)
  [2020-02-22 16:17:07.000] 1: Received 200 buffers:   393.1 KB (buffer average size:   2.0 KB) in   0.09 seconds (4299.4 KB/second)

This will tell you the link speed in KB/sec – kilobytes per second. If you want
kilobits, multiply by ``8``.

You can use the ``make menuconfig`` to reverse the setup, and have nuttx be the
server, and the host be the client. If you do that, start the server first
(nuttx), then start the client (host).
