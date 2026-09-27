==============================
Read-ahead and write buffering
==============================

A layer that sits between a driver and the block device under it, so that a
stream of small accesses becomes a smaller number of large ones.  The driver
does not implement any of it; it hands its sector read and write functions
to this layer and lets it decide when to talk to the hardware.

The code is in ``drivers/misc/rwbuffer.c``, and the interface is in
``include/nuttx/drivers/rwbuffer.h``.

What it does
============

``CONFIG_DRVR_WRITEBUFFER``
   Collects writes in memory and flushes them as one larger write.  This
   matters most on flash, where a write is preceded by an erase: turning
   four sector writes into one can be the difference between usable and
   unusable.

``CONFIG_DRVR_READAHEAD``
   When a sector is read, reads the ones after it too, on the assumption
   that a file being read sequentially will want them.  A hit is answered
   from memory; a miss cost one larger read instead of one small one.

Both are opt-in, and both trade RAM for throughput.  There are two gates,
not one: the configuration symbol above, and the buffer size the driver
fills in when it sets up -- ``wrmaxblocks`` and ``rhmaxblocks`` in ``struct
rwbuffer_s``.  A size of zero blocks turns that half off even in a build
that enabled it.  The driver's ``wrflush`` and ``rhreload`` callbacks are
still used in that case, just without a buffer behind them.

Three smaller options ride along: ``CONFIG_DRVR_READBYTES`` adds a byte read
method, ``CONFIG_DRVR_REMOVABLE`` handles media that can be taken out, and
``CONFIG_DRVR_INVALIDATE`` adds cache invalidation.

What it costs
=============

Write buffering means **data that the application believes is written may
still be in RAM**, and until it is flushed a power loss loses it.

``CONFIG_DRVR_WRDELAY`` -- 350 milliseconds by default -- is what eventually
pushes it out, but read what it measures: the timer restarts on *every*
write, so it fires only after that long with **no write activity at all**.
It is an idle timeout, not a bound on how long data may sit in the buffer.
A process writing steadily can keep the flush postponed indefinitely.
Setting the delay to zero removes the timed flush entirely.

The flush runs on the low-priority work queue, so
``CONFIG_SCHED_WORKQUEUE`` is required; without it the file refuses to
compile.

This is the same bargain every operating system makes, and the same answer
applies: if it matters, flush it, and understand that flushing is what costs
the time buffering saved.

Read-ahead is cheaper to reason about -- a wrong guess wastes a little time
and some RAM, and nothing is lost -- but it is still RAM that a small system
may not have.
