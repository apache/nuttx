=======================
Keyboard/Keypad Drivers
=======================

**Keypads vs. Keyboards** Keyboards and keypads are really the
same devices for NuttX. A keypad is thought of as simply a
keyboard with fewer keys.

**Special Commands**. In NuttX, a keyboard/keypad driver is simply
a character driver that may have an (optional) encoding/decoding
layer on the data returned by the character driver. A keyboard may
return simple text data (alphabetic, numeric, and punctuation) or
control characters (enter, control-C, etc.) when a key is pressed.
We can think about this the "normal" keyboard data stream.
However, in addition, most keyboards support actions that cannot
be represented as text or control data. Such actions include
things like cursor controls (home, up arrow, page down, etc.),
editing functions (insert, delete, etc.), volume controls, (mute,
volume up, etc.) and other special functions. In this case, some
special encoding may be required to multiplex the normal text data
and special command key press data streams.

**Key Press and Release Events** Sometimes the time that a key is
released is needed by applications as well. Thus, in addition to
normal and special key press events, it may also be necessary to
encode normal and special key release events.

**Encoding/Decoding** Layer. An optional encoding/decoding layer
can be used with the basic character driver to encode the keyboard
events into the text data stream. The function interfaces that
comprise that encoding/decoding layer are defined in the header
file ``include/nuttx/input/kbd_code.h``. These functions provide
an matched set of (a) driver encoding interfaces, and (b)
application decoding interfaces.

#. **Driver Encoding Interfaces**. These are interfaces used by
   the keyboard/keypad driver to encode keyboard events and data.

   -  ``kbd_press()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``ch``: The character to be added to the output stream.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_release()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``ch``: The character associated with the key that was
         released.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_specpress()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``keycode``: The command to be added to the output
         stream. The enumeration ``enum kbd_keycode_e keycode``
         identifies all commands known to the system.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

   -  ``kbd_specrel()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``keycode``: The command to be added to the output
         stream. The enumeration ``enum kbd_keycode_e keycode``
         identifies all commands known to the system.
      -  ``stream``: An instance of ``lib_outstream_s`` to perform
         the actual low-level put operation.

      **Returned Value:**

#. **Application Decoding Interfaces**. These are user interfaces
   to decode the values returned by the keyboard/keypad driver.

   -  ``kbd_decode()``

      **Function Prototype:**

      **Description:**

      **Input Parameters:**

      -  ``stream``: An instance of ``lib_instream_s`` to perform
         the actual low-level get operation.
      -  ``pch``: The location to save the returned value. This
         may be either a normal, character code or a special
         command (i.e., a value from ``enum kbd_getstate_s``.
      -  ``state``: A user provided buffer to support parsing.
         This structure should be cleared the first time that
         ``kbd_decode()`` is called.

      **Returned Value:**

      -  ``KBD_PRESS`` (0)**: Indicates the successful receipt
         of normal, keyboard data. This corresponds to a keypress
         event. The returned value in ``pch`` is a simple byte of
         text or control data.
      -  ``KBD_RELEASE`` (1)**: Indicates a key release event.
         The returned value in ``pch`` is the byte of text or
         control data corresponding to the released key.
      -  ``KBD_SPECPRESS`` (2)**: Indicates the successful
         receipt of a special keyboard command. The returned value
         in ``pch`` is a value from ``enum kbd_getstate_s``.
      -  ``KBD_SPECREL`` (3)**: Indicates a special command key
         release event. The returned value in ``pch`` is a value
         from ``enum kbd_getstate_s``.
      -  ``KBD_ERROR`` (``EOF``)**: An error has getting the
         next character (reported by the ``stream``). Normally
         indicates the end of file.

**I/O Streams**. Notice the use of the abstract I/O streams in
these interfaces. These stream interfaces are defined in
``include/nuttx/streams.h``.

