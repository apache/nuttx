======================
Pinctrl Device Drivers
======================

- The Pinctl driver framework allows applications and drivers to flexibly configure
  and manage pin parameters such as functionality, strength, driver type, and slewrate
  (voltage transition speed). This framework significantly enhances the flexibility
  and configurability of the system in terms of hardware interface control.

-  ``include/nuttx/pinctrl/pinctrl.h``
   All structures and APIs needed to work with pinctrl drivers are provided in
   this header file.

-  ``struct pinctrl_dev_s`` and ``struct pinctrl_ops_s``.
   Each pinctrl device driver must implement an instance of ``struct pinctrl_dev_s``.
   And the ``struct pinctrl_ops_s`` defines a call table with the following methods:

   #. **set_function**: Configures the pin's multiplexing (Mux) function, allowing it
      to be set as a specific hardware interface (e.g., UART, SPI, I2C) or as a
      general-purpose GPIO pin.
   #. **set_strength**: Allows the user to configure the pin's drive strength to meet
      the requirements of different hardware interfaces.
   #. **set_driver**: Controls the pin's driver type, such as push-pull output or
      open-drain output.
   #. **set_slewrate**: Enables the configuration of pin slew rate, which is crucial
      for high-speed digital signal transmission, optimizing signal rise and fall times.
   #. **select_gpio**: Configures the pin function as GPIO.

- Convenience macros are provided to map these operations directly:
  ``PINCTRL_SETFUNCTION``,``PINCTRL_SETSTRENGTH``,``PINCTRL_SETDRIVER``,``PINCTRL_SETSLEWRATE``,
  ``PINCTRL_SELECTGPIO``.

- Application developers can configure and control pins by opening /dev/pinctrl0 nodes
  and using the ioctl system call.
  cmd: PINCTRLC_SETFUNCTION, PINCTRLC_SETSTRENGTH, PINCTRLC_SETDRIVER, PINCTRLC_SETSLEWRATE,
  PINCTRLC_SELECTGPIO.
  parameters: struct pinctrl_param_s.

