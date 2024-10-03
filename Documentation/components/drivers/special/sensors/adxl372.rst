ADXL372
=======

Contributed by Bob Feretich

The ADXL372 is a 200g tri-axis accelerometer that is capable of detecting
and recording shock impact impact events. Recording trigger
characteristics are programmed into the sensor via multiple threshold and
duration registers.  The ADXL372 is a SPI only device that can transfer
data at 10 MHz.  The data transfer performance of this part permits the
sensor to be sampled "on demand" rather than periodically sampled by a
worker task.

See the description of the "Common Sensor Register Interface" below for more
details. It also implements the "Sensor Cluster Driver Interface".
