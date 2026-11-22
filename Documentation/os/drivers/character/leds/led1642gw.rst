=========
LED1642GW
=========

This is a driver for the STMicroelectronics LED1642GW, a 16-channel constant
current LED driver with a 4-wire serial interface and 12/16-bit PWM
brightness control for each channel. The driver is a PWM lower-half driver
with one PWM channel for each output.

Driver Configuration
====================

To use the driver, enable the following option in menuconfig:

- ``Device Drivers -> LED Support -> LED1642GW LED Driver``
  (``CONFIG_LEDS_LED1642GW``)

``CONFIG_PWM_NCHANNELS`` sets how many outputs can be changed with one
request.

Driver Usage
============

Board-specific logic must call ``led1642gw_initialize`` during the board
initialization sequence. This function takes the board configuration, writes
the configuration register, turns all outputs off and returns the PWM
lower-half instance. The instance can be registered with ``pwm_register``
or used by other drivers, e.g. ``rgbled_register``.

The serial interface is bit-banged because the command is selected by the
number of CLK rising edges with LE high at the end of each 16-bit word.
The board configuration provides the functions that set the state of the
CLK, SDI and LE lines, the current gain, the current range and the
brightness resolution. The board must also provide a free-running clock on
the PWCLK pin: the PWM frequency of the outputs is the PWCLK frequency
divided by 4096 (12-bit mode) or by 65536 (16-bit mode).

.. code-block:: c

    #include <nuttx/leds/led1642gw.h>
    #include <nuttx/leds/rgbled.h>

    static const struct led1642gw_config_s g_config =
    {
      .clk   = board_led1642gw_clk,
      .sdi   = board_led1642gw_sdi,
      .le    = board_led1642gw_le,
      .gain  = LED1642GW_GAIN_MAX,
      .range = false,
      .pwm16 = false,
    };

    /* In your board's initialization logic */

    pwm = led1642gw_initialize(&g_config);

    /* RGB LED on outputs 0, 1 and 2 */

    rgbled_register("/dev/rgbled0", pwm, pwm, pwm, 1, 2, 3);

The PWM channel ``n`` is the output ``n - 1`` and the duty is the brightness
of the output. A duty of 0 turns the output off. The outputs that are not in
the channel list of a request are not changed. The requested PWM frequency is
ignored. Stopping the PWM turns all outputs off.

The error detection and the SDO line are not supported.
