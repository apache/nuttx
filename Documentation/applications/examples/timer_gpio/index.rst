``timer_gpio``
==============

This example uses the timer interrupt to periodically change the state of a
digital output. The digital output may be a relay, a led or anything else.
This example can be very useful to validate timer drivers by using a logic
analyzer connected to the digital output. This example mainly differs from
the timer example because it waits on a sigwaitinfo() instead of using a
signal handler. This approach ensures a deterministic wake-up time when the
signal occurs.

Dependencies:

- ``CONFIG_TIMER`` – The timer driver must be selected.
- ``CONFIG_DEV_GPIO`` – The GPIO driver must be selected.

Note: You should also select one timer instance and have the gpio driver
properly configured in your board logic.

Example configuration:

- ``EXAMPLES_TIMER_GPIO_TIM_DEVNAME`` – This is the name of the timer device
    that will be used. Default: ``/dev/timer0``.
- ``EXAMPLES_TIMER_GPIO_GPIO_DEVNAME`` – This is the name of the gpio device
    that will be used. Default: ``/dev/gpio0``.
- ``EXAMPLES_TIMER_GPIO_INTERVAL`` – This is the timer interval in
    microseconds. Default: ``1000000``.
- ``EXAMPLES_TIMER_GPIO_SIGNO`` – This is the signal number that is used to
    notify that a timer interrupt occurred. Default: ``32``.
- ``EXAMPLES_TIMER_GPIO_STACKSIZE`` – This is the stack size allocated when the
    timer task runs. Default: ``2048``.
- ``EXAMPLES_TIMER_GPIO_PRIORITY`` – This is the priority of the timer task.
  Default: ``255``.
- ``EXAMPLES_TIMER_GPIO_PROGNAME`` – This is the name of the program that will
    be used from the nsh. Default: ``timer_gpio``.
