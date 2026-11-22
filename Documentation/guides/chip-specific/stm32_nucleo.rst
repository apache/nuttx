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
IKS01A2    ``CONFIG_BOARD_STM32_IKS01A2``      X-NUCLEO-IKS01A2 sensors, see
                                               `Shields`_
=========  ==================================  ================================

The file systems are not mounted when ``CONFIG_DISABLE_MOUNTPOINT`` is
set. The IWDG uses ``STM32_LSI_FREQUENCY`` from the board ``board.h``.

Timers
------

Timer and oneshot devices are registered from one option per instance,
N being the TIM or LPTIM instance number:

============================================  ===================
Option                                        Device
============================================  ===================
``CONFIG_BOARD_STM32_COMMON_TIMn_TIMER``      ``/dev/timerN``
``CONFIG_BOARD_STM32_COMMON_TIMn_ONESHOT``    ``/dev/oneshotN``
``CONFIG_BOARD_STM32_COMMON_LPTIMn_TIMER``    ``/dev/lptimerN``
``CONFIG_BOARD_STM32_COMMON_LPTIMn_ONESHOT``  ``/dev/lponeshotN``
============================================  ===================

The options need ``CONFIG_TIMER`` or ``CONFIG_ONESHOT`` (a TIM oneshot
also ``CONFIG_STM32_ONESHOT``) and the ``CONFIG_STM32_TIMn`` or
``CONFIG_STM32_LPTIMn`` instance. An instance is either a timer or a
oneshot. A TIM oneshot counts in microseconds. An LPTIM counts on its
kernel clock, ``STM32_LPTIMn_CLKIN`` in the board ``board.h``.

Rules for boards:

- Do not repeat any of the features above in the board directory.
- Board-independent initialization is added to
  ``stm32_common_initialize()``, not to individual boards.
- ``board.h`` defines ``STM32_LPTIMn_CLKIN`` for every LPTIM instance of
  the chip.

Shields
=======

Shields on the Arduino connector are supported by the common board
code: enable the shield option and the I2C instance of the board's
Arduino connector.

X-NUCLEO-IKS01A2
----------------

Motion MEMS and environmental sensor shield, enabled with
``CONFIG_BOARD_STM32_IKS01A2``. The sensors are on the I2C bus
``CONFIG_BOARD_STM32_IKS01A2_I2CBUS`` (default 1) and are registered as
uORB topics, N being ``CONFIG_BOARD_STM32_IKS01A2_DEVNO`` (default 0):

=========  =======  =============================================
Sensor     Address  Topics
=========  =======  =============================================
LSM6DSL    0x6b     ``sensor_accelN``, ``sensor_gyroN``
HTS221     0x5f     ``sensor_humiN``, ``sensor_ambient_tempN``
LPS22HB    0x5d     ``sensor_baroN``
LSM303AGR  0x19     ``sensor_accelN+1``
LSM303AGR  0x1e     ``sensor_magN+1``
=========  =======  =============================================

The LSM6DSL is register-compatible with the LSM6DS3TR-C and uses its
driver.

Boards with an ``iks01a2`` configuration: nucleo-f303re, nucleo-f767zi,
nucleo-g491re, nucleo-h533re, nucleo-h743zi and nucleo-n657x0-q.
