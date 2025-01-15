==============
Sensor Drivers
==============

Currently in NuttX we have 3 different approaches to sensor interfaces:

.. toctree::
    :maxdepth: 1

    sensors/sensors_uorb.rst
    sensors/sensors_legacy.rst
    sensors/sensors_cluster.rst

The preferred way for implementing new sensors is
the :ref:`New sensor framework <new_sensor_framework>`, which provides the most
general interafce.

.. attach files to avoid warinigs, but don't show them here !

.. toctree::
    :hidden:

    sensors/adt7320.rst
    sensors/adxl345.rst
    sensors/adxl362.rst
    sensors/adxl372.rst
    sensors/aht10.rst
    sensors/ak09912.rst
    sensors/lsm330.rst
    sensors/mcp9600.rst
    sensors/mpl115a.rst
    sensors/sht4x.rst
