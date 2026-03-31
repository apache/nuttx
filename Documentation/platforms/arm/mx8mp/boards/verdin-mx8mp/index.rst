=============
verdin-mx8mp
=============

This directory provides board support for the Toradex Verdin i.MX8MP. This
port runs on the internal Cortex-M7 auxiliary core, not on the main
Cortex-A53 complex.

GPIO Connections
================

Connect LEDs and buttons to the header pins as follows:

* ``LED21`` on ``GPIO_4``
* ``LED22`` on ``GPIO_3``
* ``LED23`` on ``GPIO_2``
* ``LED24`` on ``GPIO_1``
* ``SW11`` on ``GPIO_5_CSI``

You can adjust this pin mapping in ``verdin_mx8mp.h``.

Status
======

* 2023-08-23: boots into NSH and provides the NSH prompt
* 2023-09-04: GPIO and I2C support work, including the on-board INA219 sensor
