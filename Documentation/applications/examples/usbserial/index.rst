====================================
``usbserial`` USB Serial Hello World
====================================

Target configuration
--------------------

This is another implementation of "Hello, World" but this one uses a USB serial
driver. Configuration options can be used to simply the test. These options
include:

-  ``CONFIG_EXAMPLES_USBSERIAL_INONLY`` – Only verify IN (device-to-host) data
   transfers. Default: both.
-  ``CONFIG_EXAMPLES_USBSERIAL_OUTONLY`` – Only verify OUT (host-to-device) data
   transfers. Default: both.
-  ``CONFIG_EXAMPLES_USBSERIAL_ONLYSMALL`` – Send only small, single packet
   messages. Default: Send large and small.
-  ``CONFIG_EXAMPLES_USBSERIAL_ONLYBIG`` – Send only large, multi-packet messages.
   Default: Send large and small.

If ``CONFIG_USBDEV_TRACE`` is enabled (or ``CONFIG_DEBUG_FEATURES`` and
``CONFIG_DEBUG_USB``), then the example code will also manage the USB trace
output. The amount of trace output can be controlled using:

- ``CONFIG_EXAMPLES_USBSERIAL_TRACEINIT`` – Show initialization events.
- ``CONFIG_EXAMPLES_USBSERIAL_TRACECLASS`` – Show class driver events.
- ``CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS`` – Show data transfer events.
- ``CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER`` – Show controller events.
- ``CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS`` – Show interrupt-related events.

Error results are always shown in the trace output.

Host-side test program
----------------------

In additional to the target device-side example, there is also a host-side
application in this directory. This host side application must be executed on a
Linux host in order to perform the ``USBSERIAL`` test. The host application can be
compiled under Linux (or Cygwin?) as follows::

  cd examples/usbserial
  make -f Makefile.host TOPDIR=<nuttx-directory>

Running the test
----------------

This will generate a small program called ``host``. Usage:

1. Build the ``examples/usbserial`` target program and start the target.

2. Wait a bit, then do enter::

     dmesg

   At the end of the dmesg output, you should see the serial device was
   successfully idenfied and assigned to a tty device, probably ``/dev/ttyUSB0``
   or ``/dev/ttyACM0`` (depending on the configured USB serial driver).

3. Then start the host application::

     ./host [<tty-dev>]

   Where:

   - ``<tty-dev>`` is the USB TTY device to use. The default is ``/dev/ttyUSB0``
     (for the PL2303 emulation) or ``/dev/ttyACM0`` (for the CDC/ACM serial
     device).

The host and target will exchange are variety of very small and very large
serial messages.
