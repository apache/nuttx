README
======

The NuttX configuration for the Olimex STM32-P407 is derives more or
less directly from the Olimex STM32-P207 board support.

Note that CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG is enabled so
that the JTAG connection is not disconnected by the idle loop.

The following peripherals are enabled in this configuration.
 - LEDs:       show the sytem status

 - Buttons:    TAMPER-button, WKUP-button, J1-Joystick (consists of RIGHT-,
               UP-, LEFT-, DOWN-, and CENTER-button). Built in app
               'buttons' works.

 - ADC:        ADC1 samples the red trim potentiometer AN_TR
               Built in app 'adc' works.

 - USB-FS-OTG: enabled but not really tested, since there is only a
               USB-A-connector (host) connected to the full speed µC inputs.
               The other connector (device) is connected to the high speed µC
               inputs, but it seems that NuttX has currently no driver
               for it.

 - CAN:        Built in app 'can' works, but appart from that not really tested.

 - Ethernet:   Ping to other station on the network works.
