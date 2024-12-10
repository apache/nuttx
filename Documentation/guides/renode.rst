===================
Run NuttX on Renode
===================

Renode (https://renode.io/) is and open source virutal development
framework dedicated for complex embedded systems.

This page contains notes on running some of NuttX boards on Renode.

stm32f4discovery
================

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

At default Renode uses UART with EasyDMA enabled (UARTE) which is not supported
by Nuttx yet. We can get around this by creating our own machine description
based on Renode default implementation::

  using "platforms/cpus/nrf52840.repl"

  uart0:
    easyDMA: false

Renode script::

  using sysbus

  mach create
  machine LoadPlatformDescription @nrf52840_custom.repl

  $bin?=@nuttx

  showAnalyzer uart0

  macro reset
  """
    sysbus LoadELF $bin
  """

  runMacro $reset

Tested with ``nrf52840-dk/nsh``.

Known issues:

* ``QSPI`` not implemented in Renode,

* ``PWM`` doesn't work, missing ``NRF52_PWM_EVENTS_SEQSTARTED0_OFFSET``
  implementation in Renode,

* ``ADC`` doesn't work, missing ``NRF52_SAADC_EVENTS_CALDONE_OFFSET``
  implementation in Renode,

* ``SoftDevice`` doesn't work, crash in ``mpsl_init()``

stm32f746g-disco
================

``CONFIG_ARMV7M_BASEPRI_WAR=y`` must be set.

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
