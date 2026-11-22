================
ST Nucleo F303RE
================

.. tags:: arch:arm, chip:stm32f3, part:stm32f303, vendor:st

The Nucleo F303RE is a member of the Nucleo-64 board family.

Configurations
==============

iks01a2
-------

NSH with an X-NUCLEO-IKS01A2 sensor shield on the Arduino connector
(I2C1).  The sensors are registered as uORB topics and can be read with
``sensortest``, the bus can be scanned with ``i2c dev``.
