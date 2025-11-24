==============
Timers Drivers
==============

.. toctree::
  :caption: Supported Drivers

  timer.rst
  oneshot.rst
  pwm.rst
  watchdog.rst
  rtc.rst
  capture.rst

The NuttX timing subsystem consists of four layers:

  .. figure:: timer.svg

  * 1 Hardware Timer Drivers: Includes implementations of various hardware
                              timer drivers.
  * 2 Timer Driver Abstraction: Such as Oneshot and Timer, which provide
                                oneshot/periodical timer hardware abstraction.
  * 3 OS Timer Interfaces: Arch_Timer(up_timer_*) and Arch_Alarm(up_alarm_*), 
                           offering relative timer (trigger an event with a certain delay)
                           and absolute timer (trigger an event at a certain time)
                           interfaces.
  * 4 OS Timer Abstraction: The wdog timer manages software timers
                            and provides a unified timer API to upper layers.

The Timer Driver Abstraction is not mandatory. If the driver is simple enough
(e.g.,just providing a periodic tick), the OS Timer interfaces Arch_Timer
can be implemented directly, bypassing the timer driver abstraction layer.

The design purpose of the Timer Driver Abstraction is to simplify driver
implementation, improve performance, and enhance the reusability of driver
code.
