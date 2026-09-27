===
NFS
===

A client for the Network File System: a directory served by another machine,
mounted so that programs read and write it as if it were local.  There is no
server side in NuttX -- the board is always the client.

Enabled with ``CONFIG_NFS``.  The code is in ``fs/nfs/``.

What is implemented
===================

**NFS version 3**, over UDP or TCP.  The protocol is in ``nfs_proto.h``, the
RPC layer that carries it in ``rpc_clnt.c``, and the VFS side -- the part
that makes it look like a file system -- in ``nfs_vfsops.c``.

The transport is not a build option: it is the ``sotype`` field of ``struct
nfs_args``, in ``include/nuttx/fs/nfs.h``, chosen when the share is mounted.
``SOCK_DGRAM`` gives UDP and ``SOCK_STREAM`` gives TCP, and ``rpc_clnt.c``
carries both.

The same structure holds the knobs that decide how the client behaves on a
poor link: ``timeo``, a timeout in deciseconds; ``retrans``, how many times
a request is resent; and ``rsize``, ``wsize`` and ``readdirsize``, the
transfer sizes.

``CONFIG_NFS`` depends on ``CONFIG_ALLOW_BSD_COMPONENTS``, because this code
came from BSD and carries that licence.  A build that must avoid BSD
licensed code cannot use it.  It also needs mount point support --
``!CONFIG_DISABLE_MOUNTPOINT`` -- and selects ``CONFIG_FS_LARGEFILE``, since
the protocol works in 64-bit offsets.

Two smaller options: ``CONFIG_NFS_STATISTICS`` collects counters, which the
Kconfig notes have no user interface and are only useful under a debugger;
and ``CONFIG_NFS_DONT_BIND_TCP_SOCKET`` exists for drivers such as the
GS2200M that cannot bind a local port for a TCP client socket.

What it is good for
===================

The thing NFS gives an embedded system is **storage the board does not have
to own**:

* logs and captured data that would not fit in flash, and that somebody will
  want to look at from a desktop anyway;
* a root file system served from a workstation during development, so that
  changing a program does not mean reflashing.

The cost is the obvious one, and it is worth being explicit about it: every
read and write is a network round trip.  A program that opens a file over
NFS and reads it a byte at a time will be very slow, and the fix is
buffering in the application, or a larger ``rsize`` and ``wsize`` at mount
time, rather than anything in the driver.  Mounted over UDP the client is
also the one responsible for retries -- ``timeo`` and ``retrans`` govern
them -- so a lossy link shows up as latency rather than as errors.

Setting one up
==============

:doc:`/guides/networking/nfs` covers the practical side: what to put in the
configuration, how to mount from NSH, and how to configure the server on the
other end.
