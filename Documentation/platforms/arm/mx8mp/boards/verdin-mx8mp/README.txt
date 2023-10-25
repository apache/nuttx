README for the verdin-mx8mp Relax
============================

  The directory provides board support for the Toradex Verdin mx8mp.
  Note: this port works on the internal Cortex-M7 auxiliary core, NOT the main Cortex-53 complex!

  LEDs and buttons shall be connected to header pins like this:
  - LED21 on GPIO_4
  - LED22 on GPIO_3
  - LED23 on GPIO_2
  - LED24 on GPIO_1
  - SW11 on GPIO_5_CSI
  You can adjust this pinout in verdin_mx8mp.h

Status
======

  2023-08-23:   The Verdin mx8mp boots into NSH, provides the NSH prompt.
  2023-09-04:   gpio support, i2c support (tested with on-board ina219 sensor)
