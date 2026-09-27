=========================
Porting and Conventions
=========================

Notes that are not about one subsystem: what a chip has to provide, how the
make build works, and the naming rules the code follows.  Two others sit here
for historical rather than topical reasons: :doc:`hardfaults`, which
:doc:`/debugging/cortexmhardfaults` covers at greater length, and
:doc:`simulation`, alongside :doc:`/guides/simulation/simulator`.

How the OS itself works is in :doc:`/os/index`.  The step-by-step guide to
adding a new SoC or board is :doc:`/guides/porting/port`.

.. toctree::
   :maxdepth: 1

   chip_h.rst
   hardfaults.rst
   make_build_system.rst
   naming_arch_mcu_board_interfaces.rst
   naming_os_internals.rst
   nuttx_initialization_sequence.rst
   simulation.rst
