================
Host File System
================

The Host file system provides a mechanism to mount directories from the host OS
during simulation mode. The host directory to be "mounted" is specified during
the mount command using the ``-o`` command line switch, such as::

			mount -t hostfs -o fs=/home/user/nuttx_root /host

For non-NSH operation, the option ``fs=home/user/nuttx_root`` would
be passed to the ``mount()`` routine using the optional ``void *data``
parameter.
