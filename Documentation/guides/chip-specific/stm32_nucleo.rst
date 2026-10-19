=====================
STM32 Nucleo standard
=====================

Status: draft, see `issue 19296
<https://github.com/apache/nuttx/issues/19296>`_.

This page collects the rules that all STM32 Nucleo boards in NuttX follow.
It starts with the common board initialization and grows as more of the
board logic becomes shared.

Serial console
==============

The default console of every Nucleo board is the USART wired to the
ST-LINK virtual COM port, so a board works over its USB cable without
any extra wiring. The pins of that USART are always defined in the
board ``board.h``, not only when a board Kconfig console choice selects
them, so a configuration that bypasses the board Kconfig (for example a
custom board build pointing at the board directory) only needs to
enable the USART and select it as the console. Other console wirings
(Arduino or Morpho headers) stay optional choices.

Common board initialization
===========================

Initialization that does not depend on the board lives in
``boards/arm/common/stm32/src/board_common.c``. It is part of the common
STM32 board logic (``CONFIG_ARCH_BOARD_COMMON``), which every Nucleo board
selects, and provides one function:

.. code-block:: c

   #include "board_common.h"

   int stm32_common_initialize(void);

Every Nucleo board calls ``stm32_common_initialize()`` early in its
bringup, in ``stm32_bringup()`` or, if the board has none, in
``board_late_initialize()``. A failure is logged and the bringup continues.

The function currently handles:

=========  ==================================  ================================
Feature    Enabled by                          Result
=========  ==================================  ================================
procfs     ``CONFIG_FS_PROCFS``                mounted at
                                               ``CONFIG_NSH_PROC_MOUNTPOINT``
                                               or ``/proc``
tmpfs      ``CONFIG_FS_TMPFS``                 mounted at
                                               ``CONFIG_LIBC_TMPDIR``
IWDG       ``CONFIG_STM32_IWDG``               ``/dev/watchdog0``
WWDG       ``CONFIG_STM32_WWDG``               ``/dev/watchdog1``
CAN        ``CONFIG_STM32_CAN_CHARDRIVER``     ``/dev/can0``, ``/dev/can1``,
           ``CONFIG_STM32_FDCAN_CHARDRIVER``   ... in instance order
SocketCAN  ``CONFIG_STM32_CAN_SOCKET``         interfaces for all enabled
           ``CONFIG_STM32_FDCAN_SOCKET``       instances
I2C        ``CONFIG_I2C_DRIVER``               ``/dev/i2cN`` for every enabled
                                               ``CONFIG_STM32_I2CN``
=========  ==================================  ================================

The file systems are not mounted when ``CONFIG_DISABLE_MOUNTPOINT`` is
set. The IWDG uses ``STM32_LSI_FREQUENCY`` from the board ``board.h``.

Rules for boards:

- Do not repeat any of the features above in the board directory.
- Board-independent initialization is added to
  ``stm32_common_initialize()``, not to individual boards.
