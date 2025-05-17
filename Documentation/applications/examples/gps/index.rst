===================
``gps`` GPS example
===================

This example can be used to interact with GPS devices in NuttX. It uses the `MINMEA <https://github.com/kosma/minmea>`_
library to parse standard NMEA messages and print out GPS data to the console.

To use the program, provide the character device path for the GPS serial connection as the only argument. If no path is
provided, the program will default to ``/dev/ttyS1``.

.. code:: console

   nsh> gps /dev/ttyS3
   Fixed-point Latitude...........: 0
   Fixed-point Longitude..........: 0
   Fixed-point Speed..............: 0
   Floating point degree latitude.: nan
   Floating point degree longitude: nan
   Floating point speed...........: nan
   Fix quality....................: 0
   Altitude.......................: 0
   Tracked satellites.............: 0

The output will show ``nan`` and ``0`` for values while waiting to obtain a fix, at which point real values will begin
to appear.

The program loops forever parsing NMEA values from the serial device.
