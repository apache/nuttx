==========
AURIX TC3X
==========

NuttX support for the Infineon AURIX TC3X (TriCore TC1.6.2) family.

The TC3X port covers SCU clock tree bring-up (PLL, CCU dividers, ENDINIT
password rotation), the IR/SRC interrupt routing, the System Timer (STM)
oneshot and ASCLIN-based UART.  Boards select a particular TC3X variant
through ``CONFIG_TC3X_CHIP_*``.

.. toctree::
   :maxdepth: 1
   :glob:

   */*
