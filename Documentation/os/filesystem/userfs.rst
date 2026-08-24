================
User file system
================

See ``include/nuttx/fs/userfs.h``.

Advisory locking and inotify
============================

The daemon may implement the optional ``getlk``/``setlk`` methods to
arbitrate advisory locks itself (``setlk`` is non-blocking; return
``-EAGAIN`` on conflict) and the ``notify`` method to learn the watch
life cycle.  Without them the OS local lock table is used.  Events are
injected back with ``userfs_notify_event()``
(``CONFIG_FS_NOTIFY_INJECT``).
