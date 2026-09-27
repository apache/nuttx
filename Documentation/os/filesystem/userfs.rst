=================
UserFS
=================

A file system implemented by an ordinary application rather than by kernel
code.  The kernel side is a stub that forwards every VFS operation to a
server task, which answers however it likes.

Enabled with ``CONFIG_FS_USERFS``.  The code is in ``fs/userfs/``.

What it is for
==============

Anything where the file system logic does not belong in the kernel:

* a file system that talks to a remote service, where the implementation
  wants to make network calls and block freely;
* a bridge to storage a vendor library already knows how to reach;
* a file system being developed, where a crash in a task is much easier to
  live with than a crash in the kernel.

The application sees ``/mnt/whatever`` and calls ``open()`` on it like any
other path.  It does not know a task is answering.

How the operations get there
============================

The server task calls ``userfs_run()``, which is part of the C library
rather than the kernel.  It mounts the file system for you, then stays in a
loop receiving requests and dispatching them to the callbacks you supplied,
and does not return until the file system is unmounted.

The two sides find each other by port number.  ``userfs_run()`` mounts with
a ``struct userfs_config_s`` holding a ``portno`` and a maximum write size;
the kernel side reads that number in its ``bind`` method and opens a
``PF_INET`` datagram socket to ``INADDR_LOOPBACK`` on it.  Server ports run
from ``0x8300`` to ``0x83ff``, so there is room for 256 of them.

That is also the answer to a configuration surprise: ``CONFIG_FS_USERFS``
depends on ``CONFIG_NET_IPv4``, ``CONFIG_NET_UDP`` and
``CONFIG_NET_LOOPBACK``.  A file system that touches no network still needs
the network stack configured, because a **local UDP socket** is the IPC it
is built on.

Twenty-three request types cover the VFS surface, from ``USERFS_REQ_OPEN``
through to ``USERFS_REQ_CHSTAT``; they are listed in ``enum userfs_req_e``.

.. note::

   ``include/nuttx/fs/userfs.h`` also declares ``userfs_register()``, to
   "register the UserFS factory driver at dev/userfs", and
   ``include/nuttx/fs/ioctl.h`` defines a ``FIONUSERFS`` command to drive
   it.  Neither is implemented: there is no definition of
   ``userfs_register()`` anywhere in the tree, and nothing consumes
   ``FIONUSERFS``.  Mounting through ``userfs_run()`` is the only route
   that exists.

What it costs
=============

Every operation is a round trip to another task: a context switch out, the
server's work, a context switch back.  For a file system whose backing store
is slow anyway -- a network, a bus, a device behind a vendor library -- that
overhead disappears into the noise.  For anything where latency matters, a
kernel file system is the right answer.
