==================================
``modbus`` FreeModbus demo example
==================================

This is a port of the FreeModbus Linux demo. It derives from the demos/LINUX
directory of the FreeModBus version ``1.5.0`` (June 6, 2010) that can be
downloaded in its entirety from
http://developer.berlios.de/project/showfiles.php?group_id=6120.

- ``CONFIG_EXAMPLES_MODBUS_PORT``, Default ``0`` (for ``/dev/ttyS0``).
- ``CONFIG_EXAMPLES_MODBUS_BAUD``, Default B``38400``.
- ``CONFIG_EXAMPLES_MODBUS_PARITY``, Default ``MB_PAR_EVEN``.
- ``CONFIG_EXAMPLES_MODBUS_REG_INPUT_START``, Default ``1000``.
- ``CONFIG_EXAMPLES_MODBUS_REG_INPUT_NREGS``, Default ``4``.
- ``CONFIG_EXAMPLES_MODBUS_REG_HOLDING_START``, Default ``2000``.
- ``CONFIG_EXAMPLES_MODBUS_REG_HOLDING_NREGS``, Default ``130``.

The FreeModBus library resides at ``apps/modbus``.
See :doc:`/applications/industry/modbus/index` for additional configuration
information.
