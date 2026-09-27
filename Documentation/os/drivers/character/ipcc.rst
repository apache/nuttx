============================================
Inter-Processor Communication Channel (IPCC)
============================================

A character device for talking to another processor on the same chip.  The
application opens ``/dev/ipccN``, writes to send and reads to receive; what
carries the bytes is hardware specific and hidden behind the driver.

The code is in ``drivers/ipcc/`` and the interface in
``include/nuttx/ipcc.h``.

.. warning::

   ``CONFIG_IPCC`` depends on ``CONFIG_EXPERIMENTAL``.  The driver cannot be
   selected without it, and the Kconfig says so in place of the option when
   it is off.

The ``N`` in ``/dev/ipccN`` is the channel the lower half was registered
with, counted from zero, not a running count of devices.  A chip whose IPCC
block has several channels gets one device per channel, and the board
decides which of them to bring up.

Why a character device
======================

Because it means no new interface to learn.  A program that already knows
how to read and write a file can talk to the other core, and everything that
already works on file descriptors -- ``poll()``, blocking and non-blocking
mode, redirection -- works here too without anything being added for it.

A board port provides the lower half: a driver that knows how the two
processors actually signal each other, which in the one port that exists
today -- the STM32WL5, in ``arch/arm/src/stm32wl5/stm32wl5_ipcc.c`` -- is a
block of shared mailbox memory plus a transmit and a receive interrupt.

The lower half is deliberately simple.  Its ``read()`` and ``write()`` must
never block, may transfer less than they were asked for, and return zero
when the mailbox is empty or full.  Everything that turns that into a file
descriptor -- waiting, ``poll()``, ``O_NONBLOCK`` returning ``-EAGAIN`` --
is the upper half's work.

Buffering is the upper half's too, but only if you ask for it.
``CONFIG_IPCC_BUFFERED``, on by default, gives each channel a circular
receive and transmit buffer whose sizes are arguments to
``ipcc_register()``; a reader is then served from the buffer instead of
waiting on the other processor.  Turn it off and those arguments disappear
from ``ipcc_register()``, the lower half drops two of its methods, and every
read and write goes straight to the mailbox.

When to use something else
==========================

IPCC is a byte pipe between two processors.  If what you need is a *service*
on the other processor -- a file system, a network stack, a clock
controller -- then :doc:`RPMSG </os/drivers/special/rpmsg/index>` is the
better answer: it has named channels and a request/response shape, and
several subsystems already speak it.
