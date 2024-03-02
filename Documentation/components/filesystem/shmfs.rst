=========================
Shared Memory File System
=========================

This supports the POSIX shm_open() APIs for shared memory among unrelated
apps.

It can be enabled with ``CONFIG_FS_SHMFS=y``. To check how it works, please
also enable the example app via ``CONFIG_EXMAPLE_SHM=y`` and run ``shm_test``
from NSH command line.

This file system doesn't support mount operations though.

If comment the line using ``shm_unlink()`` in the example app, we can see
a file under ``/var/shm/`` from NSH command line after running the example.
We can also remove that file from command line.

