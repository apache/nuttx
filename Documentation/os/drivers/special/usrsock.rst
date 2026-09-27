============================
User-space sockets (usrsock)
============================

A call to ``socket()`` can be answered by a daemon running as an ordinary
task instead of by the NuttX network stack.  Turn on ``CONFIG_NET_USRSOCK``
and the order reverses: ``psock_socket()`` offers **every** new socket to
the daemon first, and only falls back to the kernel stack if the daemon
declines it.

That fallback is the interesting part, and it is per socket.  The daemon
answers ``-ENOSYS`` or ``-ENOTSUP`` for a socket it does not want, and that
one is created against the kernel stack as usual.  If the daemon was never
started, or has died, the setup fails with ``-ENETDOWN`` and the same
fallback happens, so a build with usrsock compiled in still works with no
daemon running.  The two stacks coexist rather than replace one another.

The work is split in two: ``net/usrsock/`` holds the implementation, one
file per socket call -- ``usrsock_connect.c``, ``usrsock_sendmsg.c`` and so
on -- while ``drivers/usrsock/`` holds only the transports that carry the
requests.  The wire protocol between the two is in
``include/nuttx/net/usrsock.h``.

What it is for
==============

The case it exists for is a modem or a Wi-Fi module that runs its own TCP/IP
stack and is spoken to over AT commands or a vendor protocol.  There is no
Ethernet frame to hand to the NuttX stack, and no point in having two
stacks.  With usrsock the application still calls ``socket()``, ``connect()``
and ``send()``, and a daemon translates each one into whatever the module
expects.

The application does not have to know.  That is the whole point: the same
program runs over a NuttX-stack Ethernet interface and over a modem, because
what changed is below the socket API.

How the request reaches the daemon
==================================

The kernel side turns each socket call into a request, and the daemon
answers it.  How the request travels is a single choice -- the three are
mutually exclusive, and the first is the default:

``CONFIG_NET_USRSOCK_DEVICE``
   Exports ``/dev/usrsock``.  The daemon opens that device, reads requests
   and writes responses.  This is the usual arrangement when the daemon runs
   on the same processor.

``CONFIG_NET_USRSOCK_RPMSG``
   Sends requests over an RPMSG channel instead, for a system where the
   stack lives on another processor.

``CONFIG_NET_USRSOCK_CUSTOM``
   Neither of the above: the board provides its own transport.

``CONFIG_NET_USRSOCK_RPMSG_CPUNAME`` names the processor the RPMSG server
runs on, and that server side is enabled separately with
``CONFIG_NET_USRSOCK_RPMSG_SERVER``.  ``CONFIG_NET_USRSOCK_PREALLOC_CONNS``,
six by default, sets how many usrsock connection structures are allocated up
front.

What it costs
=============

Every socket operation now crosses into another task and back.  For a modem
talking at serial speeds that is irrelevant -- the link is the bottleneck by
orders of magnitude.  For anything fast it is not, and the local stack is
the better answer.
