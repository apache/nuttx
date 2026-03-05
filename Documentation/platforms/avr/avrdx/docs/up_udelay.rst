=============================================
AVR DA/DB family ``up_udelay`` implementation
=============================================

NuttX provides functions for busy sleep, these are documented
:doc:`here </reference/os/sleep>`. These functions
use ``BOARD_LOOPSPERMSEC`` configuration value to determine how many loops need
to be done to cause requested delay. Creator of a the board code is supposed
to calibrate this value to make the delay as precise as possible.

This does not match well with AVR DA/DB microcontrollers because the CPU clock
frequency is configurable and unless the user chooses one and sticks with it,
any value in ``BOARD_LOOPSPERMSEC`` is going to be incorrect.

Instead of using the default ``up_udelay`` function, custom one is provided.
When called, it determines current clock settings and infers required loop count
from that.

Configuration
=============

This implementation is enabled automatically and can be turned off using
:menuselection:`System Type --> Use AVR DA/DB implementation of up_udelay`
configuration option. If enabled, there are two additional options:

:menuselection:`External clock is not supported in up_udelay` and
:menuselection:`32.768kHz oscillator is not supported in up_udelay`.

The latter simply excludes the code that checks if any 32.768kHz clock source
is in use.

The former does the same thing for external clock but also has an additional
effect: when not selected, it does not set ``ARCH_HAVE_DYNAMIC_UDELAY``
configuration option. This in turn means that ``BOARD_LOOPSPERMSEC`` will
need to be configured
in :menuselection:`System Type --> Delay loops per millisecond`

The reason for this is that with the external clock, there is no way of knowing
what the current CPU clock frequency is and it is therefore impossible
to calculate the loop count. It needs to be provided by board code's author
to match the external clock the board is using. If the main clock prescaler is
active, the loop value is recalculated to take that into consideration.

Note that ``BOARD_LOOPSPERMSEC`` also needs to be specified if this ``up_udelay``
implementation is disabled altogether.

The other functions - ``up_mdelay`` and ``up_ndelay`` - are unchanged. These
simply multiply or divide their time parameter by 1000 and pass the result
to ``up_udelay``.

Precision
=========

The loop count calculation takes time and that time depends on current settings
and requested wait time. For example, calculation for wait time below
180 microseconds when using high frequency oscillator can be done using 16bit
arithmetic and without expensive division - unless the compiler is set
to optimize for code size.

On the other hand - whenever the main clock prescaler is in effect,
division is unavoidable.

The function attempts to account for this by guessing how much time all
the processing took and reduce the loop count accordingly. Nevertheless,
the wait may be considerably longer than requested for shorter delays
and lower clock speeds.

(For example - 32.768kHz clock with prescaler of 2 needs to do two
divisions, taking 42 milliseconds total.)
