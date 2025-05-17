===================
``gps`` GPS example
===================

This example can be used to interact with GPS devices in NuttX. It uses the
`MINMEA <https://github.com/kosma/minmea>`_ library to parse standard NMEA
messages and print out GPS data to the console.

To use the program, provide the character device path for the GPS serial
connection as the only argument. If no path is provided, the program will
default to ``/dev/ttyS1``.

If you want to be able to see the floating point output of this program, make
sure you remember to enable ``CONFIG_LIBC_FLOATINGPOINT``.

The program loops forever parsing NMEA values from the serial device.

Waiting for GPS Fix
-------------------

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

The output will show ``nan`` and ``0`` for values while waiting to obtain a fix,
at which point real values will begin to appear.

With GPS Fix
------------

.. code:: console

   nsh> gps /dev/ttyS3
   Altitude.......................: 73172                                         
   Tracked satellites.............: 5                                             
   Fixed-point Latitude...........: 4628356                                       
   Fixed-point Longitude..........: -8058408                                      
   Fixed-point Speed..............: 110                                           
   Floating point degree latitude.: 46.476547                                     
   Floating point degree longitude: -80.977995                                    
   Floating point speed...........: 0.001833                                      
   Fix quality....................: 1                

You can now see the information is filled in with the data from the GPS.

.. note::

   Fixed-point readings may have a different scale than the floating-point
   readings. For instance, the altitude above is fixed-point, but is actually
   73.172 meters in floating point.
