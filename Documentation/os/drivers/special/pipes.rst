===========================
FIFO and named pipe drivers
===========================

A pipe is a byte stream with a writer at one end and a reader at the other.
NuttX implements them as character drivers, so a pipe is read and written
with the same ``read()`` and ``write()`` any other file uses, and a thread
blocked on an empty pipe is in the same ``WAIT_SEM`` state as any other
blocked thread.

The code is in ``drivers/pipes/``.

Two kinds
=========

``pipe()``
   An anonymous pipe.  It returns two file descriptors and has no name in
   the file system, so only the process that created it -- and anything that
   inherits its descriptors -- can reach it.  It does briefly have one: the
   driver is registered under ``CONFIG_DEV_PIPE_VFS_PATH``, ``/var/pipe`` by
   default, both descriptors are opened from there, and the name is then
   unregistered while the open descriptors keep the pipe alive.

``mkfifo()``
   A named pipe, created at whatever path you hand it -- there is no
   ``/dev`` in the driver -- which any task can then open.  That is how two
   unrelated programs use one.

Both are POSIX interfaces and behave as POSIX describes them; see
:doc:`/reference/user/10_filesystem` for the calls themselves.

Buffering and blocking
======================

The whole subsystem is enabled by ``CONFIG_PIPES``.  Within it, pipes and
FIFOs are sized separately: ``CONFIG_DEV_PIPE_SIZE`` and
``CONFIG_DEV_FIFO_SIZE`` each set a default ring buffer in bytes -- 1024, or
256 under ``CONFIG_DEFAULT_SMALL`` -- and each disables its own half when
set to zero, so a build can have FIFOs without pipes or the other way
round.  ``CONFIG_DEV_PIPE_MAXSIZE``, 65535 by default, caps what a program
may ask for at runtime.

The size is worth choosing rather than accepting, because it decides when
each side blocks:

* a reader blocks while the buffer is empty, unless the pipe was opened with
  ``O_NONBLOCK``, in which case it gets ``-EAGAIN``;
* a writer blocks while the buffer is full, for the same reason.

Both sides wait on a ``sem_t`` of their own -- ``d_rdsem`` and ``d_wrsem``
in ``struct pipe_dev_s`` -- through ``nxsem_wait()``, which is what puts the
thread in ``TSTATE_WAIT_SEM``.  ``poll()`` works too, with
``CONFIG_DEV_PIPE_NPOLLWAITERS`` setting how many threads may wait at once.

A buffer that is too small turns a producer and a consumer into a pair of
threads that hand the CPU back and forth on every message.  A buffer that is
too large is memory that sits idle in a system that usually does not have
much of it.
