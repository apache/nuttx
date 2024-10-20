===========
Semihosting
===========

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Semihosting


Relevant files:

.. code-block:: bash

    fs/hostfs/
    arch/arm/include/armv7-m/syscall.h
    arch/arm/src/common/up_hostfs.c

Mounting:

.. code-block:: bash

    mount -t hostfs -o fs=/host/path /local/path