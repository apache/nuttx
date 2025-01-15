=====================
Sensor Legacy Drivers
=====================

The old sensors implementation where the character device interface is not
standardized in any way.

This approach is not recommended for new drivers, because without a
standarized interface, creating portable application is imposible.

Implemented Drivers
===================

Drivers that are available also with the new sensor framework are marked with ``[*]``.

- :doc:`adt7320`
- adxl345
- :doc:`aht10`
- :doc:`ak09912`
- amg88xx
- apds9922
- apds9960
- as5048a
- as5048b
- as726x
- bh1749nuc [*]
- bh1750fvi
- bmg160
- bmi088 [*]
- bmi160 [*]
- bmi270 [*]
- bmp180 [*]
- dhtxx
- fxos8700cq
- hall3ph
- hc_sr04
- hdc1008
- hts221
- ina219
- ina226
- ina3221
- isl29023
- kxtj9
- lis2dh
- lis331dl
- lis3dh
- lis3dsh
- lis3mdl
- lm75
- lm92
- lps25h
- lsm303agr
- lsm6dsl
- lsm9ds1
- ltc4151
- max31855
- max31865
- max44009
- max6675
- mb7040
- :doc:`mcp9600`
- mcp9844
- mlx90393
- mlx90614
- :doc:`mpl115a`
- mpu60x0
- ms58xx
- msa301
- qencoder
- scd30
- scd41
- sgp30
- sht21
- sht3x
- :doc:`sht4x`
- sps30
- t67xx
- veml6070
- vl53l1x
- xen1210
- zerocross
