===================
Run NuttX on Renode
===================

Renode (https://renode.io/) is and open source virutal development
framework dedicated for complex embedded systems.

This page contains notes on running some of NuttX boards on Renode.

ARM-v7m
=======

Renode doesn't correctly handle ``SVC`` instruction escalation to HardFault
when ``PRIMASK=1`` which crashs NuttX in the first ``up_exit()`` call.
We can work around this problem by enabling BASEPRI::

  CONFIG_ARMV7M_USEBASEPRI=y

stm32f4discovery
================

``CONFIG_ARMV7M_USEBASEPRI=y`` must be set.

Renode doesn't support CCM memory, so we have to disable it
with ``CONFIG_MM_REGIONS=1``.

Renode script::

  using sysbus
  $name?="STM32F4_Discovery"
  mach create $name
  machine LoadPlatformDescription @platforms/boards/stm32f4_discovery-kit.repl

  cpu PerformanceInMips 125

  $bin?=@nuttx

  showAnalyzer sysbus.usart2

  macro reset
  """
    sysbus LoadELF $bin
  """

  runMacro $reset


Tested with ``stm32f4discovery/nsh``.

nucleo-l073rz
=============

Doesn't work. No BASEPRI implementation for ``Cotex-M0`` in NuttX.

nrf52840-dk
===========

Nordic UART peripheral is not supported by Renode, UARTE support
is required (EasyDMA).

stm32f746g-disco
================

``CONFIG_ARMV7M_USEBASEPRI=y`` and ``CONFIG_ARMV7M_BASEPRI_WAR=y`` must be set.

Renode script::

  using sysbus
  $name?="STM32F746"
  mach create $name
  machine LoadPlatformDescription @platforms/boards/stm32f7_discovery-bb.repl

  $bin ?= @nuttx

  showAnalyzer usart1
  showAnalyzer ltdc

  macro reset
  """
    sysbus LoadELF $bin
  """

  runMacro $reset

Tested with ``stm32f746g-disco/nsh``.

Known issues:

* ``stm32f746g-disco/lvgl`` - crash due to incompatible I2C our touchscreen driver

nucleo-h743zi
=============

``CONFIG_ARMV7M_USEBASEPRI=y`` must be set.

Renode doesn't support ``PWR_CSR1_ACTVOSRDY`` bit so we have to disable
it with ``CONFIG_STM32H7_PWR_IGNORE_ACTVOSRDY=y``.

Renode script::

  using sysbus
  mach create "nucleo_h743zi"
  include @platforms/boards/nucleo_h753zi.repl

  $bin=@nuttx

  showAnalyzer sysbus.usart3

  macro reset
  """
    sysbus LoadELF $bin
  """

  runMacro $reset

Tested wtih ``nucleo-h743zi/nsh``.
