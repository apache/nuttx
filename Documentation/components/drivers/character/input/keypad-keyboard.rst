=======================
Keyboard/Keypad Drivers
=======================


**Keypads vs. Keyboards** Keyboards and keypads are really the same
devices for NuttX. A keypad is thought of as simply a keyboard with
fewer keys.

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

Writing a Keyboard Driver
=========================

A keyboard driver registers with the keyboard upper half, which owns the
character device and the buffering. This is the whole contract, and it is
what makes a keyboard work with every application that reads one: the
``kbd`` dump, the LVGL terminal, NXDoom, and anything written later.

.. code-block:: c

   #include <nuttx/input/keyboard.h>

   struct mykbd_dev_s
   {
     struct keyboard_lowerhalf_s lower;  /* Must be found by container_of */
     ...
   };

   /* Register once, at board bring-up.  The open, close and write callbacks
    * of the lower half are all optional:  a driver that has nothing to do
    * when the device is opened can register with the structure zeroed.
    */

   keyboard_register(&priv->lower, "/dev/kbd0", CONFIG_MYKBD_NBUFFER);

   /* Report each key as it changes state */

   keyboard_event(&priv->lower, keycode, type);

Do not store the driver state in ``lower.priv``. ``keyboard_register()``
overwrites that field with the upper half's own state, so a lower half
callback recovers the driver state with ``container_of()`` instead.

Reporting the Right Event Type
------------------------------

``type`` has **four** values, and getting this wrong is a silent failure:
the build is clean and the symptom is a key that does nothing.

======================  =====  =========================================
Type                    Value  ``code`` carries
======================  =====  =========================================
``KEYBOARD_PRESS``      0      The character that the key produces
``KEYBOARD_RELEASE``    1      The character of the key that came up
``KEYBOARD_SPECPRESS``  2      A value from ``enum kbd_keycode_e``
``KEYBOARD_SPECREL``    3      A value from ``enum kbd_keycode_e``
======================  =====  =========================================

A key that produces a character is reported with its character and the
plain press and release types. A key that does not, such as an arrow, a
function key or a modifier, is reported with a keycode and one of the SPEC
types.

The distinction matters because the two ranges overlap. ``KEYCODE_UP`` is
7 and so is the ASCII bell, so an application that receives a 7 has no way
to tell them apart other than by the event type. A driver that reports
every key as ``KEYBOARD_PRESS`` will have its arrow keys silently dropped
by any application that follows this contract.

Choosing the Device Name
------------------------

Register as ``/dev/kbd0``. Applications look for a keyboard under that
name, and a keyboard registered anywhere else is out of reach of all of
them unless the user knows to point each one at it by hand.

The USB HID driver is the exception: it names its devices ``/dev/kbda``
onwards, because they come and go as keyboards are plugged in and more
than one can be present at a time.

Matrix Keyboards
----------------

A matrix keyboard does not need a driver of its own. ``INPUT_KMATRIX``
scans the matrix, debounces it and reports the events, so a board provides
a keymap and four GPIO callbacks and nothing more. See
``include/nuttx/input/kmatrix.h``.

Wrap a keymap entry in ``KMATRIX_SPECIAL()`` to declare that it holds a
keycode rather than a character, so that the key is reported with a SPEC
event type:

.. code-block:: c

   static const uint32_t g_keymap[] =
   {
     '1', '2', '3',
     KMATRIX_SPECIAL(KEYCODE_UP), KMATRIX_SPECIAL(KEYCODE_DOWN), '0',
   };

Testing a New Keyboard
----------------------

``system/kbd`` prints every event that a keyboard reports, whatever the
hardware behind it is, so it is the first thing to run on a new one::

    nsh> kbd /dev/kbd0
    kbd: reading /dev/kbd0, keyboard events
    press     code  97 'a'
    release   code  97 'a'
    specpress keycode 7
    specrel   keycode 7

An application can also be brought up against a keyboard that does not
exist yet. ``UINPUT_KEYBOARD`` registers ``/dev/ukeyboard``, whose events
are injected by writing ``struct keyboard_event_s`` to it, and ``kbd -i``
injects whatever it reads from its own stdin.

.. note::

   The leading u is for uinput, not for USB. It marks a device that
   software feeds rather than hardware, and the same prefix names
   ``/dev/utouch``, ``/dev/ubutton`` and ``/dev/usensor``. A USB keyboard
   is ``/dev/kbda``, with no prefix at all.

::

    nsh> kbd -i /dev/ukeyboard
    kbd: injecting into /dev/ukeyboard, type to send, Ctrl-D to stop

Given a second device it forwards every key of one keyboard onto another
instead of reading its own stdin, so a real keyboard and an injected one
can drive the same application::

    nsh> kbd -i /dev/ukeyboard /dev/kbd0 &
    kbd: forwarding /dev/kbd0 into /dev/ukeyboard

An application cannot do that on its own, since it opens a single device.

Point an application at ``/dev/ukeyboard`` and it is driven from the serial
console, or from whatever is on the other end of it. This is the way to
work on the application side of a board port before its keyboard driver
exists, and the way to reach a board whose keyboard is not at hand.

Raise ``UINPUT_KEYBOARD_BUFNUMBER`` before doing so. It counts events
rather than keys, so the default of eight holds four keystrokes, and a
console hands over a whole line at once. The upper half overwrites the
oldest event when the buffer is full, so what arrives is the tail of the
line and nothing says that the rest was dropped.

Byte Stream Compatibility
=========================

An application that has not been converted to read events can select
``INPUT_KEYBOARD_BYTESTREAM``, which makes every keyboard deliver the byte
stream described below instead of events. This is a property of the build
rather than of the driver, so it applies to every keyboard at once and a
configuration cannot mix the two.

The byte stream cannot express a key release, and it cannot express a
special key at all unless ``LIBC_KBDCODEC`` is enabled. An application
that needs either has to read the events.

**Encoding/Decoding** Layer. An optional encoding/decoding layer
can be used with the basic character driver to encode the keyboard
events into the text data stream. The function interfaces that
comprise that encoding/decoding layer are defined in the header
file ``include/nuttx/input/kbd_code.h``. These functions provide
a matched set of (a) driver encoding interfaces, and (b)
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

