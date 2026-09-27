====================
Asynchronous I/O
====================

Ordinary ``read()`` and ``write()`` block: the calling thread waits until
the transfer is finished.  Asynchronous I/O submits the transfer and returns
immediately, leaving the thread free to do something else while it
completes.

Enabled with ``CONFIG_FS_AIO``, which provides the interfaces declared in
``include/aio.h``.  The code is in ``fs/aio/``.

How it works in NuttX
=====================

NuttX does not implement asynchronous I/O in the driver layer.  It runs the
ordinary blocking operation on the **low-priority work queue**, and notifies
the caller when that thread is done.  That is why ``CONFIG_FS_AIO`` depends
on ``CONFIG_SCHED_WORKQUEUE``: without a worker thread there is nowhere for
the transfer to happen.

It also depends on signals not being disabled, because a signal is how
completion is reported back.

Running on the low-priority queue would normally mean a high-priority thread
gets its I/O done at low-priority speed.  It does not, because the queue is
boosted: ``aio_queue()`` calls ``lpwork_boostpriority()`` with the
submitting thread's priority before queueing the work, and restores it
afterwards, so the transfer runs at the priority of whoever asked for it.

The consequence worth understanding is that asynchronous I/O here buys
**concurrency, not speed**.  The transfer takes exactly as long as it would
have; what changes is that your thread is not the one waiting.  On a system
with one CPU and a driver that is already interrupt-driven, that may be no
gain at all -- the thread would have been blocked and some other thread
would have run anyway.  It pays off when a thread has real work to do while
a slow device is busy.

Configuration
=============

``CONFIG_FS_NAIOC``
   How many AIO containers are pre-allocated, eight by default.  Each
   in-flight operation uses one, so this is the number of transfers that may
   be outstanding at the same time.

   Ask for one more and the submitting call **blocks until a container frees
   up** -- ``aio_read()``, ``aio_write()`` and ``aio_fsync()`` all wait on a
   counting semaphore initialised to this number, and the wait is
   uninterruptible.  It is worth sizing deliberately, because it is the one
   way an asynchronous call becomes a blocking one: exceed the count and the
   thread waits, which is the thing the interface exists to avoid.
