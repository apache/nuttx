``pwm`` General PWM Example
===========================

A test of a PWM device driver. It simply enables a pulsed output for a specified
frequency and duty for a specified period of time. This example can ONLY be
built as an NSH built-in function.

This test depends on these specific PWM/NSH configurations settings (your
specific PWM settings might require additional settings).

- ``CONFIG_PWM`` – Enables PWM support.
- ``CONFIG_PWM_PULSECOUNT`` – Enables PWM pulse count support (if the hardware
  supports it).
- ``CONFIG_NSH_BUILTIN_APPS`` – Build the PWM test as an NSH built-in function.

Specific configuration options for this example include:

- ``CONFIG_EXAMPLES_PWM_DEVPATH`` – The path to the default PWM device. Default:
  ``/dev/pwm0``.
- ``CONFIG_EXAMPLES_PWM_FREQUENCY`` – The initial PWM frequency. Default: ``100`` Hz
- ``CONFIG_EXAMPLES_PWM_DUTYPCT`` – The initial PWM duty as a percentage. Default:
  ``50%``.
- ``CONFIG_EXAMPLES_PWM_DURATION`` – The initial PWM pulse train duration in
  seconds. Used only if the current pulse count is zero (pulse count is only
  supported if ``CONFIG_PWM_PULSECOUNT`` is defined). Default: ``5`` seconds.
- ``CONFIG_EXAMPLES_PWM_PULSECOUNT`` – The initial PWM pulse count. This option is
  only available if ``CONFIG_PWM_PULSECOUNT`` is non-zero. Default: ``0`` (i.e., use
  the duration, not the count).
