=================
RPMSG File System
=================

Once RPMsg link is available, we can use rpmsg file system to mount remote directories with the help of RPMsg file system drivers.

Here we show an example of mounting and using a master file system path from remote side, it is as easy as using a local file system.

Building
========

At file system server side (the master), we need enable the ``CONFIG_FS_RPMSGFS_SERVER`` configuration. 

At file system client side (the remote), we need enable the ``CONFIG_FS_RPMSGFS`` configuration.

Then we build the two sides accordingly.

Running
=======

Using the following command to mount the master's ``/proc`` file system to ``/proc.master`` from the ``nsh`` sessino of the remote node.

.. code:: console

  remote> mount -t rpmsgfs -o cpu=master,fs=/proc /proc.master
  remote> cat /proc/uptime /proc.master/uptime 
        39.06                                                                      
        39.06                                                                      
  remote>

Note the ``-o cpu=master,fs=/proc`` specifies the ``master`` node's ``/proc`` path as the source, the ``/proc.master`` is the mount point at remote side. All files under that mount point is actually hosted at the master side. The ``-t rpmsgfs`` selects the RPMsg file system driver to serve the operation.

