ADXL345
=======

Contributed by Alan Carvalho de Assis

The ADXL345 accelerometer can operate in I2C or SPI mode. To operate in I2C
mode just connect the CS pin to Vddi/o.

In order to operate in SPI mode CS need to use connected to microcontroller,
it cannot leave unconnected.

In SPI mode it works with clock polarity (CPOL) = 1 and clock phase (CPHA)
= 1.
