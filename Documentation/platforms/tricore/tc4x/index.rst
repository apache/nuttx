==========
AURIX TC4X
==========

NuttX support for the Infineon AURIX TC4X (TriCore TC1.8) family.

The TC4X port covers SCU clock tree bring-up (PLL, CCU dividers,
SafetyEndInit/CpuEndInit unlock), the IR/SRC interrupt routing
(SRPN equals the IRQ number), the System Timer (STM) oneshot and
ASCLIN-based UART.  Boards select a particular TC4X variant through
``CONFIG_TC4X_CHIP_*``.

.. toctree::
   :maxdepth: 1
   :glob:

   */*
