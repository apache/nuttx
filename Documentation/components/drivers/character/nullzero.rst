=================================
``dev_null.c`` and ``dev_zero.c``
=================================

These files provide the standard ``/dev/null`` and ``/dev/zero`` devices.  See
``include/nuttx/drivers/drivers.h`` for prototypes of functions that should
be called if you want to register these devices (``devnull_register()``
and ``devzero_register()``).
