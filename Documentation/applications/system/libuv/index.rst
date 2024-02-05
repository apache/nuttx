========================================
``libuv`` libuv asynchronous I/O Library
========================================

Most features of libuv are supported by current port, except SIGPROF relative function (loop_configure).

Nearly full libuv's test suite available on NuttX, but some known case can't run on sim:

* ``loop_update_time``
* ``idle_starvation``
* ``signal_multiple_loops``
* ``signal_pending_on_close``
* ``metrics_idle_time``
* ``metrics_idle_time_thread``
* ``metrics_idle_time_zero``

And some will cause crash by some reason:

* ``fs_poll_ref``
