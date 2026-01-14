V9FS
====

V9FS is a remote file system based on the 9P2000.L protocol.

Adding V9FS to the NuttX Configuration
======================================

The V9FS client is easy to add to your configuration. Just add
``CONFIG_FS_V9FS`` to ``nuttx/.config``.

In order to fully run V9FS, you also need to select a transport layer
driver. The two currently available are:

  - **VIRTIO** -> ``CONFIG_V9FS_VIRTIO_9P=y``
  - **SOCKET** -> ``CONFIG_V9FS_SOCKET_9P=y``

NFS Mount Command
=================

In V9FS, we have some special parameters

  - ``uname``. Used to indicate the user identity of the client
  - ``aname``. Optional, it specifies the file tree that the client requests to access
  - ``trans``. Selects the transport layer (virtio/socket)
  - ``msize``. The maximum size of the message
  - ``tag``. The tag of the mount point

Different transport layers have different requirements for parameter
passing. Here are some examples:

Qemu + VIRTIO
--------------

.. code-block:: console

  mount -t v9fs -o trans=virtio,tag=<mount_tag> /dir

Similarly, we need to bring the corresponding parameters in qemu

.. code-block:: console

  -fsdev local,security_model=none,id=fsdev1,path=<share-path> \
  -device virtio-9p-device,id=fs1,fsdev=fsdev1,mount_tag=<mount_tag>

For how to start virtio-9p in QEMU, please refer to the document:

  - https://wiki.qemu.org/Documentation/9psetup



Socket
-------

.. code-block:: console

  mount -t v9fs -o trans=socket,tag=<IP Address>:[Port Default 563],aname=[path] /dir

There are many types of 9P socket servers. Here we use R9-fileserver
(a cross-platform 9p server based on Rust
https://github.com/crafcat7/R9-fileserver)

.. code-block:: console

  sudo ./ya-vm-file-server --network-address <IP Address>:<Server Port> --mount-point <share-path>


Result
------

.. code-block:: fish

  NuttShell (NSH)
  nsh> mkdir mnt
  nsh> 
  nsh> ls mnt
  /mnt:
  nsh> mount -t v9fs -o trans=virtio,tag=hostshare /mnt/v9fs
  nsh> 
  nsh> ls /mnt/v9fs
  /mnt/v9fs:
  sdcard/
  mnt/
  nsh> 
  nsh> echo "This is a test" >/mnt/v9fs/testfile.txt
  nsh> ls -l /mnt/v9fs
  /mnt/v9fs:
  drwxrwxrwx    1000    1000        4096 sdcard/
  -rw-rw-rw-    1000    1000          15 testfile.txt
  drwxrwxrwx    1000    1000        4096 mnt/
  nsh> 
  nsh> cat /mnt/v9fs/testfile.txt
  This is a test
  nsh> 
