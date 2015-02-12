ADXL345
=======

The ADXL345 accelerometer can operate in I2C or SPI mode. To operate in I2C
mode just connect the CS pin to Vddi/o.

In order to operate in SPI mode CS need to use connected to microcontroller,
it cannot leave unconnected.

In SPI mode it works with clock polarity (CPOL) = 1 and clock phase (CPHA) = 1.

MPL115A
=======

This driver has support only for MPL115A1 (SPI), but support to MPL115A2 (I2C) can
be added easily.
