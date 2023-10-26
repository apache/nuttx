============
sama5d2-xult
============

This is the  port of NuttX to the Atmel SAMA5D3x-EK development boards
(where x=1,3,4, or 5).  These boards feature the Atmel SAMA5D3
microprocessors.  Four different SAMA5D3x-EK kits are available

- SAMA5D31-EK with the ATSAMA5D1 (http://www.atmel.com/devices/sama5d31.aspx)
- SAMA5D33-EK with the ATSAMA5D3 (http://www.atmel.com/devices/sama5d33.aspx)
- SAMA5D34-EK with the ATSAMA5D4 (http://www.atmel.com/devices/sama5d34.aspx)
- SAMA5D35-EK with the ATSAMA5D5 (http://www.atmel.com/devices/sama5d35.aspx)

The each consist of an identical base board with different plug-in modules
for each CPU.  An option 7 inch LCD is also available.  All four boards
are supported by NuttX with a simple reconfiguration of the processor
type.

For details look at ``Documentation/platforms/arm/sama5/boards/sama5d2-xult/README.txt``

.. this breaks latexpdf build
..
   .. include:: README.txt
      :literal:
