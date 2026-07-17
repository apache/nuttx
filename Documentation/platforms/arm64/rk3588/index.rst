===============
Rockchip RK3588
===============

.. tags:: arch:arm64, chip:rk3588, vendor:rockchip, experimental

.. warning::

   The support for this chip is experimental. Not all features are
   implemented and they have not been extensively tested by many users.

   Help is wanted if you are interested in supporting a feature or if you've
   found an issue with any of the implementation! See :doc:`the contributing
   guidelines </contributing/index>`.

The `Rockchip RK3588 <https://www.rock-chips.com/uploads/pdf/2022.8.26/192/RK3588%20Brief%20Datasheet.pdf>`_
is an octa-core SoC designed by Rockchip, featuring quad Cortex-A76 cores and quad Cortex-A55 cores.
It is used on single-board computers like the Orange Pi 5 Pro.

- **CPU:** Octa-core ARMv8.2 (4x Cortex-A76 up to 2.4 GHz + 4x Cortex-A55 up to 1.8 GHz)
- **Interrupt Controller:** GICv3 (GIC600)

Supported Peripherals
=====================

======================== =======
Peripheral               Support
======================== =======
UART                     Inherited UART2 debug console.
GICv3                    Yes
Timer                    ARM Generic System Timer.
PSCI                     Yes (System Reset/Reboot via SMC calls to TF-A).
GPIO                     No
SPI                      No
I2C                      No
USB                      No
Ethernet                 No
SDMMC                    No
======================== =======

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
