=========
abnt_codi
=========

The ABNT CODI is an old energy meter standard used in Brazil.

This code interprets the end user serial output existent in the energy meter.
That output externalizes its data blinking an LED as a serial protocol at the
baudrate of ``110 BPS`` and uses ``8`` octects:

=======   =======   =================================================================
Octet     Bits      Description
=======   =======   =================================================================
``001``   ``0-7``   Number of seconds to the end of current active demand (``LSB``)
``002``   ``0-3``   Number of seconds to the end of current active demand (``MSB``)
          ``4``     Bill indicator. It is inverted at each demand replenishment
          ``5``     Reactive Interval Indicator. Inverted at end react interval
          ``6``     If ``1`` means the reactive-capacitive is used to calculate ``UFER``
          ``7``     If ``1`` means the reactive-inductive is used to calculate ``UFER``
``003``   ``0-3``   Current seasonal segment:  ``0001`` – tip, ``0010`` – out of tip, ``1000`` – reserved
          ``4-5``   Type of charging indicator (flag): ``00`` – Blue, ``01`` – Green, ``10`` – Irrigators, ``11`` – Other
          ``6``     Not used
          ``7``     If equal ``1`` means reactive rate is enabled
``004``   ``0-7``   Number of pulses for active energy of cur dem interv (``LSB``)
``005``   ``0-6``   Number of pulses for active energy of cur dem interv (``MSB``)
          ``7``     Not used
``006``   ``0-7``   Number of pulses for reactive energy of cur dem interv (``LSB``)
``007``   ``0-6``   Number of pulses for reactive energy of cur dem interv (``MSB``)
          ``7``     Not used
``008``   ``0-7``   Inverted bits of _xor_ from previous octects
=======   =======   =================================================================
