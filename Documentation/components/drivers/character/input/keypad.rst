=======================
Matrix Keypad (KMATRIX)
=======================

**What is a Keypad?**
A keypad is a small keyboard with a limited set of keys, typically
arranged in a matrix. It is commonly used for numeric input, access
control, or simple user interfaces.

For example, a typical 12-key numeric keypad looks like this:

.. image:: images/keypad-example.png
  :alt: Example of a 12-key matrix keypad
  :align: center
  :width: 200px

**Purpose**. The KMATRIX driver provides a generic keypad
implementation for boards that expose a switch matrix through GPIOs.
It periodically scans rows and columns, detects state changes with a
simple debounce, and emits keyboard events through the common keyboard
upper-half. This makes the device available as a character driver
(e.g., ``/dev/keypad0``) using the standard keyboard
interfaces.

**Why Polling**. This first version uses polling to be broadly usable
on any board with available GPIOs, without requiring per-board IRQ
wiring, pin interrupt capabilities, or expander-specific interrupt
support. Polling also simplifies early bring-up and makes the driver
predictable while the keymap and GPIO configuration are validated.
Future iterations are expected to add interrupt-driven scanning and
I2C expander variants; the GPIO polling path remains a good baseline
and fallback.

**Driver Overview**. The KMATRIX lower-half scans the matrix and calls
``keyboard_event()`` when it detects a press or release. The keyboard
upper-half registers the character device at the requested ``devpath``
and stores events in a circular buffer. Applications read
``struct keyboard_event_s`` from the device or use the optional
kbd-codec layer.

**Board Support**. To support KMATRIX, a board must provide:

#. **GPIO Definitions**

   - Define the row and column GPIOs (arrays of pins).
   - Provide a keymap array indexed by ``row * ncols + col``.

#. **Configuration Callbacks**

   - ``config_row(pin)``: Configure a row GPIO as output.
   - ``config_col(pin)``: Configure a column GPIO as input with pull-up
     or pull-down consistent with the wiring.
   - ``row_set(pin, active)``: Drive a row active/inactive. For the
     STM32F4Discovery example, rows are driven low to activate.
   - ``col_get(pin)``: Read a column and return ``true`` when pressed.

#. **Registration Hook**

   - Implement ``board_kmatrix_initialize(const char *devpath)`` to
     call ``kmatrix_register(&config, devpath)``.
   - Invoke the board hook during bring-up (for example,
     ``board_kmatrix_initialize("/dev/keypad0")``).

**Reference Implementation (STM32F4Discovery)**. The current reference
is in ``boards/arm/stm32/common/src/stm32_kmatrix_gpio.c``:

- Rows: ``BOARD_KMATRIX_ROW0..3`` (outputs)
- Columns: ``BOARD_KMATRIX_COL0..2`` (inputs with pull-up)
- Keymap: 4x3 phone keypad layout
- Callbacks: ``km_stm32_config_row``, ``km_stm32_config_col``,
  ``km_stm32_row_set``, ``km_stm32_col_get``
- Registration: ``board_kmatrix_initialize()`` calls
  ``kmatrix_register()``

**Data Path Summary**.

- Board calls ``board_kmatrix_initialize("/dev/keypad0")``
- ``kmatrix_register()`` configures GPIOs and calls
  ``keyboard_register(&lower, devpath, buflen)``
- The upper-half registers the device node at ``devpath``
- ``kmatrix_scan_worker()`` calls ``keyboard_event()`` on press/release
- Applications read events from the device node

