==============
AVR ATMega2560
==============

Extension of the AVR architecture to support the ATMega2560 and specific support
for the Arduino Mega2560 board were contributed by Dimitry Kloper and first
released in NuttX-7.14.

Peripheral Support
------------------

The following list indicates peripherals supported in NuttX:

==========  ======= =====
Peripheral  Support Notes
==========  ======= =====
GPIO        Yes
PWM         No
ADC         No
RTC         No
WTD         No
I2C         No
UART        Yes
SPI         No
TIMER       Yes
UART        Yes
==========  ======= =====

----
UART
----

UART is implemented using interrupts. The chip doesn't support DMA.

-----
Timer
-----

The timer peripheral is exposed as standard timer.

Flashing the Device
-------------------

ATMega boards may vary in how to flash the device, but a common way to do so is
by using the ``avrdude`` utility.

The ``avrdude`` utility allows you to select the device being flashed with the
``-p`` flag. In the case of the ATMega2560, you will need to supply ``-p
m2560``.

You can also select which type of programmer you want to flash the device using.
There are a lot of options, which can be listed using ``avrdude -c ?``. A common
programmer is ``stk500v2``, which is often built into Arduino boards and allows
you to flash the device with a serial connection. You'll need to specify the
port on your computer that the programmer/device is connected to with ``-P``,
such as ``-P /dev/ttyACM0``.

Here is an example command for flashing the ``nuttx.hex`` image to the
:doc:`boards/arduino-mega2560/index`:

.. code:: console

   $ avrdude -c stk500v2 -p m2560 -P /dev/ttyACM0 -U flash:w:nuttx.hex -v -D

Supported Boards
----------------

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
