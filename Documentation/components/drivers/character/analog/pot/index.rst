======================
Digital Potentiometers
======================

The digital potentiometer driver is split into an "upper half" character
driver (``drivers/analog/pot.c``) and chip-specific "lower half" drivers.
The application interface is defined in ``include/nuttx/analog/pot.h``.

Device model
============

A potentiometer is modeled as a set of wipers, each with a position in
``[0, max]``. Optional capabilities are advertised in ``pot_info_s.caps``
so applications can discover what a given chip supports instead of
probing commands:

- ``POT_CAP_READBACK`` - wiper position can be read back
- ``POT_CAP_MOVE`` - relative wiper move (the only interface available on
  up/down-pin devices without readback, e.g. Renesas X9C10x)
- ``POT_CAP_ENABLE`` - wiper enable/shutdown
- ``POT_CAP_NV`` - non-volatile wiper store/recall

If ``pot_info_s.rab`` (terminal A-B resistance in ohms) is non-zero, the
wiper resistance is ``ohms = val * rab / max``. The value is supplied by
board logic to the lower half initializer, since the resistance variant
of a chip is not runtime-discoverable.

Application interface
=====================

All commands take a pointer argument. Optional commands return
``-ENOTSUP`` when the lower half does not implement them.

======================== ========================== =====================
IOCTL                    Argument                   Description
======================== ========================== =====================
``POTIOC_GET_INFO``      ``struct pot_info_s *``    Device properties
``POTIOC_SET_WIPER``     ``struct pot_wiper_s *``   Set wiper position
``POTIOC_GET_WIPER``     ``struct pot_wiper_s *``   Get wiper position
``POTIOC_MOVE_WIPER``    ``struct pot_move_s *``    Move wiper by signed
                                                    number of steps
``POTIOC_SET_ENABLE``    ``struct pot_enable_s *``  Enable or shut down
                                                    a wiper
``POTIOC_STORE_WIPER``   ``struct pot_nv_s *``      Store wiper to a
                                                    non-volatile slot
``POTIOC_RECALL_WIPER``  ``struct pot_nv_s *``      Recall wiper from a
                                                    non-volatile slot
======================== ========================== =====================

Vendor-specific features (e.g. per-terminal switches, register access,
wiper lock) are not part of the common interface: lower halves expose
them through chip-specific ioctl commands forwarded via ``po_ioctl``.

Supported chips
===============

========== ===== ====================================================
Chip       Bus   Notes
========== ===== ====================================================
MCP445X    I2C   Quad wiper, 257 taps. Terminal control via the
                 ``ANIOC_MCP445X_*`` ioctls, see
                 ``include/nuttx/analog/mcp445x.h``.
========== ===== ====================================================
