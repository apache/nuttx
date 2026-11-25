===========================
FIFO and named pipe drivers
===========================

Configuration
=============

- ``CONFIG_PIPES``: Enables FIFO and named pipe drivers.
- ``CONFIG_DEV_PIPE_SIZE``: Sets the default size of the pipe ringbuffer in
  bytes (defaults to 1024, or 256 on ``CONFIG_DEFAULT_SMALL=y``).
- ``CONFIG_DEV_FIFO_SIZE``: Sets the default size of the FIFO ringbuffer in
  bytes (defaults to 1024, or 256 on ``CONFIG_DEFAULT_SMALL=y``).
- ``CONFIG_DEV_PIPE_NPOLLWAITERS``: Maximum number of threads that can wait
  for POLL events on a pipe, FIFO, or local domain socket. Defaults to 2 to
  minimize dynamic memory overhead.
