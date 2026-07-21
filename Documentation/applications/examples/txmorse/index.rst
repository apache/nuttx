==================================
``txmorse`` Morse code transmitter
==================================

This application is an example which transmits Morse code messages using the
:doc:`Morsey library </applications/audioutils/morsey/index>`.

You can run ``txmorse -h`` to display help information.

Supported Devices
-----------------

.. note::

   This application does not currently support all possible Morse code
   transmission devices available on NuttX NuttX. Currently, only GPIO devices
   are supported.

In order to add support for a new output device, you must submit a patch
modifying the program to define the interface for the Morsey library for each
type of output device. Take a look at the current application code to see
examples for console devices and GPIO devices.

The ``txfunc`` function is used by the ``morsey`` library to transmit a mark.
You can implement the appropriate driver operations (i.e. ``ioctl`` calls) in
this function. See the GPIO implementation for an example.

The ``cleanupfunc`` function is used to cleanup the implementation. For most
NuttX devices, this involves closing the file descriptor.

In the ``main`` function, the path to the output device is used to identify the
implementation to use. For instance, GPIO devices are under ``/dev/gpio<n>``, so
device paths containing "gpio" cause the GPIO driver to be selected. If no
player can be associated with the device, the default player is selected which
just prints the Morse code to the console.

Usage
-----

The following will output "SOS" over a GPIO device (i.e. buzzer or LED):

.. code:: console

   nsh> txmorse "sos" /dev/gpio0
