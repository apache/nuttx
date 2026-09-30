=====
ZipFS
=====

Zipfs is a read only file system that mounts a zip file as a NuttX file system through the NuttX VFS interface.
This allows users to read files while decompressing them, without requiring additional storage space.

CONFIG
======

.. code-block:: bash

    CONFIG_FS_ZIPFS=y
    CONFIG_LIB_ZLIB=y

Example
=======

1. ``./tools/configure.sh sim:zipfs`` build sim platform with zipfs support.

2. ``make`` build NuttX.

3. ``./nuttx`` run NuttX.

4. ``nsh> mount -t hostfs -o /home/<your host name>/work /host`` mount host directory to ``/host``.

5. ``nsh> mount -t zipfs -o /host/test.zip /zip`` mount zip file to ``/zipfs``.

6. Use cat/ls command to test.

.. code-block:: bash

    nsh> ls /zip
    /zip:
     a/1
     a/2
    nsh> cat /zip/a/1
    this is zipfs test 1
    nsh> cat /zip/a/2
    this is zipfs test 2

