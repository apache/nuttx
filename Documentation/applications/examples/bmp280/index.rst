==========================================
``bmp280`` BMP280 Barometer sensor example
==========================================

This example is made for testing the BMP280 barometer sensor. It works by
reading a single measurement from the device (assuming it is registered at
``/dev/uorb/sensor_baro0``) and prints the results to the screen. The program is
run without any command line arguments.

Here is an example of the console output:

.. code-block:: console

   nsh> bmp280
   Absolute pressure [hPa] = 983.099976
   Temperature [C] = 24.129999
