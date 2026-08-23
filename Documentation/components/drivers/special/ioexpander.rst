==========================
IO Expander Device Drivers
==========================

The IO Expander subsystem is defined in the following headers:

- ``include/nuttx/ioexpander/ioexpander.h`` — defines the public IO expander
   interface: macros, types, and helper access macros used by drivers and
   consumers.
- ``include/nuttx/ioexpander/gpio.h`` — provides the "gpio lower half"
   helper that allows registering an IO expander pin as a standard GPIO
   character device (see ``gpio_lower_half`` and ``gpio_lower_half_byname``).

Each IO expander driver must implement an instance of ``struct
ioexpander_ops_s``. That structure defines the lower-half call table and
the operations a driver must provide; the public header also includes
helper macros that dispatch to the lower-half operations table.

The helper ``gpio_lower_half`` can be used to register individual expander
pins as standard GPIO devices so that upper-half GPIO consumers can access
expander pins through the common GPIO character driver.

**Binding IO expander drivers**

IO expander drivers are usually bound by board-specific code rather than
accessed directly from application code. For I2C- or SPI-connected
expanders the typical sequence is:

#. Obtain the bus instance (for example, a ``struct i2c_master_s *``) from
   the hardware-specific bus driver.
#. Call the expander driver's initialization routine with the bus instance
   and device-specific configuration; the init routine returns a
   ``struct ioexpander_dev_s *`` instance.
#. Use the returned ``ioe`` instance directly, or register individual
   expander pins with the upper-half GPIO driver via ``gpio_lower_half``.


-  **Examples**: ``drivers/ioexpander/pca9555.c``,
   ``drivers/input/aw86225.c``,
   ``drivers/analog/lmp92001.c``,
   ``drivers/ioexpander/ioe_rpmsg.c``,
   ``boards/sim/sim/sim/src/sim_ioexpander.c``,
   ``boards/arm/nrf52/thingy52/src/nrf52_sx1509.c`` etc.

Supported devices
=================

The following IO expander drivers are available under
``drivers/ioexpander/``. Each is enabled by its own Kconfig option (all
of them depend on ``CONFIG_IOEXPANDER``) and declares its initialization
routine in the header of the same name under
``include/nuttx/ioexpander/``.

.. list-table::
   :header-rows: 1
   :widths: 40 10 14 36

   * - Device
     - Bus
     - Pins
     - Kconfig option
   * - Awinic AW9523B
     - I2C
     - 16
     - ``CONFIG_IOEXPANDER_AW9523B``
   * - WCH CH422G
     - I2C
     - 12
     - ``CONFIG_IOEXPANDER_CH422G``
   * - iC-Haus iC-JX
     - SPI
     - 16
     - ``CONFIG_IOEXPANDER_ICJX``
   * - ISO1H812G
     - SPI
     - 8
     - ``CONFIG_IOEXPANDER_ISO1H812G``
   * - ISO1I813T
     - SPI
     - 8
     - ``CONFIG_IOEXPANDER_ISO1I813T``
   * - Microchip MCP23008 / MCP23S08
     - I2C
     - 8
     - ``CONFIG_IOEXPANDER_MCP23X08``
   * - Microchip MCP23017 / MCP23S17
     - I2C
     - 16
     - ``CONFIG_IOEXPANDER_MCP23X17``
   * - NXP PCA9538
     - I2C
     - 8
     - ``CONFIG_IOEXPANDER_PCA9538``
   * - NXP PCA9555
     - I2C
     - 16
     - ``CONFIG_IOEXPANDER_PCA9555``
   * - NXP PCA9557
     - I2C
     - 8
     - ``CONFIG_IOEXPANDER_PCA9557``
   * - PCF8574
     - I2C
     - 8
     - ``CONFIG_IOEXPANDER_PCF8574``
   * - PCF8575
     - I2C
     - 16
     - ``CONFIG_IOEXPANDER_PCF8575``
   * - Diodes PI4IOE5V6408
     - I2C
     - 8
     - ``CONFIG_IOEXPANDER_PI4IOE5V6408``
   * - Semtech SX1509
     - I2C
     - 16
     - ``CONFIG_IOEXPANDER_SX1509``
   * - TCA6408 / TCA6416 / TCA6424 / PCAL6416A
     - I2C
     - 8 / 16 / 24 / 16
     - ``CONFIG_IOEXPANDER_TCA64XX``

Notes on individual drivers:

- ``CONFIG_IOEXPANDER_TCA64XX`` and ``CONFIG_IOEXPANDER_PCF8574`` additionally depend on
  ``CONFIG_EXPERIMENTAL``.
- ``CONFIG_IOEXPANDER_CH422G`` presents the eight bi-directional pins, IO0-IO7, as
  pins 0-7 and the four open-drain outputs, OC0-OC3, as pins 8-11, so
  ``CONFIG_IOEXPANDER_NPINS`` must be at least 12.  The device selects a
  register by the I2C address a transfer is addressed to rather than by a
  register address written ahead of the data, and none of its write-only
  registers can be read back, so the driver shadows them.  IO0-IO7 share a
  single direction control in the hardware: the driver records the direction
  asked of each pin and puts the group in output mode once at least one of
  them is an output.  Reading a pin of a group held in output mode reports
  the value last written, because the device cannot report the pin level.
- Drivers with a ``<device>_MULTIPLE`` option support more than one
  instance of the same chip on a board.
- Drivers with a ``<device>_SHADOW_MODE`` option keep the output and
  configuration registers cached in RAM instead of performing
  read-modify-write cycles over the bus, and those with a
  ``<device>_RETRY`` option retransmit on I2C errors.
- Per-device interrupt support is enabled with ``<device>_INT_ENABLE``,
  which selects ``CONFIG_IOEXPANDER_INT_ENABLE``. Some drivers also
  offer ``<device>_INT_POLL`` and ``<device>_INT_POLLDELAY`` to poll for
  missed interrupts.

In addition to the physical devices, the following drivers implement the
same lower-half interface without directly driving a chip:

Note that ``CONFIG_IOEXPANDER_NPINS`` determines the width of
``ioe_pinset_t`` and must be at least as large as the pin count of the
device in use.

Further details
===============

Header files
------------

The relevant header files are:

- ``include/nuttx/ioexpander/ioexpander.h`` — defines macros, types and access
   macros used to interact with IO expanders.
- ``include/nuttx/ioexpander/gpio.h`` — provides the "gpio lower half" helper
   that allows registering an IO expander pin as a standard GPIO device.

Overview of key macros and options
----------------------------------

The following is a concise reference of the important macros defined in the
header. These are the options you will typically use through ``IOEXP_SETOPTION``
and the various access macros. The primary preprocessor definitions are
listed below (C syntax):

.. code-block:: c

   /* Direction definitions */
   #define IOEXPANDER_DIRECTION_IN            0  /* float */
   #define IOEXPANDER_DIRECTION_IN_PULLUP     1
   #define IOEXPANDER_DIRECTION_IN_PULLDOWN   2
   #define IOEXPANDER_DIRECTION_OUT           3  /* push-pull */
   #define IOEXPANDER_DIRECTION_OUT_OPENDRAIN 4
   #define IOEXPANDER_DIRECTION_OUT_LED       5  /* LED output */

   /* Pinset mask helpers */
   #define IOEXPANDER_PINMASK  (((ioe_pinset_t)1 << CONFIG_IOEXPANDER_NPINS) - 1)
   #define PINSET_ALL          (~((ioe_pinset_t)0))

   /* Common option values (used with IOEXP_SETOPTION) */
   /* Invert (active level) */
   #define IOEXPANDER_OPTION_INVERT      1
   #define IOEXPANDER_VAL_NORMAL         0  /* normal polarity */
   #define IOEXPANDER_VAL_INVERT         1  /* inverted polarity */

   /* Interrupt configuration (level/edge and high/low/rising/falling/both) */
   #define IOEXPANDER_OPTION_INTCFG      2
   #define IOEXPANDER_VAL_DISABLE        0  /* 0000 disable interrupts */
   #define IOEXPANDER_VAL_LEVEL          1  /* xx01: level triggered */
   #define IOEXPANDER_VAL_EDGE           2  /* xx10: edge triggered */
   #define IOEXPANDER_VAL_HIGH           5  /* 0101: high level */
   #define IOEXPANDER_VAL_LOW            9  /* 1001: low level */
   #define IOEXPANDER_VAL_RISING         6  /* 0110: rising edge */
   #define IOEXPANDER_VAL_FALLING        10 /* 1010: falling edge */
   #define IOEXPANDER_VAL_BOTH           14 /* 1110: both edges */

   /* LED configuration */
   #define IOEXPANDER_OPTION_LEDCFG      3  /* assign an LED number to a pin */

   /* Non-generic (driver-specific) option */
   #define IOEXPANDER_OPTION_NONGENERIC  4  /* pass driver-specific struct */

   /* Wakeup configuration (configure pin as SoC wake-up source) */
   #define IOEXPANDER_OPTION_WAKEUPCFG   5
   #define IOEXPANDER_WAKEUP_DISABLE     0
   #define IOEXPANDER_WAKEUP_ENABLE      1

   /* Debounce and interrupt mask (recent additions) */
   #define IOEXPANDER_OPTION_SETDEBOUNCE 6  /* configure debounce */
   #define IOEXPANDER_DEBOUNCE_DISABLE  0
   #define IOEXPANDER_DEBOUNCE_ENABLE   1

   #define IOEXPANDER_OPTION_SETMASK     7  /* control interrupt masking */
   #define IOEXPANDER_MASK_DISABLE       0  /* unmask (enable) interrupts */
   #define IOEXPANDER_MASK_ENABLE        1  /* mask (suppress) interrupts */

Access macros (API)
-------------------

The header exposes a set of helper macros that dispatch to the underlying
driver operations table (``struct ioexpander_ops_s``):

.. c:macro:: IOEXP_SETDIRECTION(dev, pin, dir)

   Set a pin direction (input, output, open-drain, LED, pull-up/down).
   Returns 0 on success or a negative errno on failure.

.. c:macro:: IOEXP_SETOPTION(dev, pin, opt, val)

   Generic option setting interface used to configure the options listed
   above. Note that ``val`` is a ``void *``; drivers may accept an integer
   casted to a pointer or a pointer to a driver-specific structure.

   Examples::

     /* Invert pin polarity */
     IOEXP_SETOPTION(dev, 3, IOEXPANDER_OPTION_INVERT,
                     (FAR void *)IOEXPANDER_VAL_INVERT);

     /* Enable debounce on pin 2 */
     IOEXP_SETOPTION(dev, 2, IOEXPANDER_OPTION_SETDEBOUNCE,
                     (FAR void *)IOEXPANDER_DEBOUNCE_ENABLE);

     /* Mask interrupts for pin 5 */
     IOEXP_SETOPTION(dev, 5, IOEXPANDER_OPTION_SETMASK,
                     (FAR void *)IOEXPANDER_MASK_ENABLE);

.. c:macro:: IOEXP_WRITEPIN(dev, pin, val)

   Set the pin level. Returns 0 on success or a negative errno on error.

.. c:macro:: IOEXP_READPIN(dev, pin, valptr)

   Read the actual physical pin level. The value is returned via ``valptr``.

.. c:macro:: IOEXP_READBUF(dev, pin, valptr)

   Read the buffered/register value cached by the expander.

   - ``IOEXP_WRITEPIN`` sets the pin level (TRUE typically means high).
     Drivers handle polarity inversion if configured.
   - ``IOEXP_READPIN`` reads the actual physical pin level.
   - ``IOEXP_READBUF`` reads the buffered/register value cached by the
     expander.

Multi-pin operations
--------------------

When ``CONFIG_IOEXPANDER_MULTIPIN`` is enabled, batch operations are
available that may be more efficient than repeated single-pin calls:

- ``IOEXP_MULTIWRITEPIN(dev, pins, vals, count)``
- ``IOEXP_MULTIREADPIN(dev, pins, vals, count)``
- ``IOEXP_MULTIREADBUF(dev, pins, vals, count)``

Interrupts and callbacks
------------------------

If ``CONFIG_IOEXPANDER_INT_ENABLE`` is enabled the header defines the
callback type and attach/detach helper macros. The callback signature
is::

   typedef CODE int (*ioe_callback_t)(FAR struct ioexpander_dev_s *dev,
                                                       ioe_pinset_t pinset, FAR void *arg);

The callback is invoked when events occur for the monitored pinset. The
attach/detach helpers are provided as macros that dispatch to the lower-half
driver when ``CONFIG_IOEXPANDER_INT_ENABLE`` is enabled:

.. c:macro:: IOEP_ATTACH(dev, pinset, callback, arg)

   Attach and enable a pin interrupt callback. Returns a non-NULL opaque
   handle on success. ``pinset`` selects which pin(s) will generate the
   callback; ``callback`` is a function of type ``ioe_callback_t`` and
   ``arg`` is passed through to the callback.

.. c:macro:: IOEP_DETACH(dev, handle)

   Detach and disable a previously attached callback referenced by ``handle``.

Note: when ``CONFIG_IOEXPANDER_NPINS`` > 64, ``ioe_pinset_t`` represents a
single interrupt pin number rather than a bitmask.

Driver interface (lower-half)
-----------------------------

Each IO expander driver must implement the operations table
``struct ioexpander_ops_s``. At minimum the driver should provide:

- ``ioe_direction``
- ``ioe_option``
- ``ioe_writepin``
- ``ioe_readpin``
- ``ioe_readbuf``

Optional multi-pin and interrupt attach/detach methods should be provided
when the corresponding configuration options are enabled.

Binding to the upper layer (gpio_lower_half)
--------------------------------------------

Applications normally do not access IO expander drivers directly. Typical
binding steps are:

1. Obtain the bus instance (for example, ``struct i2c_master_s *``) from
    the hardware-specific bus driver.
2. Call the expander driver's initialization routine with the bus instance
    and device configuration to obtain a ``struct ioexpander_dev_s *``.
3. Use the returned ``ioe`` instance directly, or register individual
    expander pins as standard GPIO devices via ``gpio_lower_half`` or
    ``gpio_lower_half_byname``.

Example (pseudocode)::

   /* Get the I2C bus */
   struct i2c_master_s *i2c = up_i2cinitialize(0);

   /* Initialize the expander (driver-specific init) */
   struct ioexpander_dev_s *ioe = pca9555_initialize(i2c, CONFIG_PCA9555_ADDR);

   /* Configure pin 0 as input with pull-up and enable debounce */
   IOEXP_SETDIRECTION(ioe, 0, IOEXPANDER_DIRECTION_IN_PULLUP);
   IOEXP_SETOPTION(ioe, 0, IOEXPANDER_OPTION_SETDEBOUNCE,
                           (FAR void *)IOEXPANDER_DEBOUNCE_ENABLE);

Examples and references
-----------------------

See the following drivers and board examples for concrete usage:

- ``drivers/ioexpander/pca9555.c`` — I2C IO expander implementation.
- ``drivers/ioexpander/ioe_rpmsg.c`` — RPMSG-based IO expander.
- ``boards/arm/nrf52/thingy52/src/nrf52_sx1509.c`` — binding example.
- ``drivers/ioexpander/pi4ioe5v6408.c`` — PI4IOE5V6408 I2C I/O expander.
