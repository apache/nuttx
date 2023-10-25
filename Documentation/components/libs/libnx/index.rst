=====
libnx
=====

The graphics capability consist both of components internal to the RTOS
and of user-callable interfaces.  In the NuttX kernel mode build there are
some components of the graphics subsystem are callable in user mode and
other components that are internal to the RTOS.  This directory, libs/libnx/,
contains only those user-callable components.

The RTOS internal functions are contained in the ``graphics/`` directory.
Please refer to ``Documentation/components/graphics`` for more detailed information.


.. toctree::
   :maxdepth: 1
   :caption: Contents:
   
   nxfonts.rst
