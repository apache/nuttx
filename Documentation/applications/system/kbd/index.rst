===================================
``kbd`` Keyboard dump and injection
===================================

Description
-----------

The ``kbd`` application prints every event that a keyboard reports. Any
keyboard registered with ``keyboard_register()`` is read the same way,
whatever the hardware behind it is, so this works with a USB HID keyboard,
a matrix, the simulator, virtio or VNC without being told which.

It is the first thing to run when bringing up a keyboard on a new board:
if the events look right here, every application that reads a keyboard
will see the same thing.

It replaces the ``hidkbd`` and ``keyboard`` examples, which did this for
one kind of keyboard each and could not read the other.

Usage
-----

.. code-block:: bash

   kbd [<device>] [<count>]
   kbd -i <device> [<source>]

Without ``-i`` it reads ``<device>`` and prints what arrives, stopping
after ``<count>`` keys if one is given. The device defaults to
``CONFIG_SYSTEM_KBD_DEVPATH``. It waits for the device to appear rather
than failing, since a USB keyboard shows up when it is plugged in.

With ``-i`` it writes instead of reading. See `Injecting keys`_ below.

Output
------

A key that produces a character is reported with the character, and a key
that does not, such as an arrow or a modifier, with a value from
``enum kbd_keycode_e``. The two ranges overlap, so the event type is what
tells them apart::

    nsh> kbd /dev/kbd0
    kbd: reading /dev/kbd0, keyboard events
    press     code  97 'a'
    release   code  97 'a'
    specpress keycode 7
    specrel   keycode 7

The four types are ``press`` and ``release`` for characters, ``specpress``
and ``specrel`` for keycodes. A driver that reports its arrow keys as
plain presses is not following the contract, and its arrows will be
dropped by applications that do.

The device delivers ``struct keyboard_event_s`` events unless the kernel
was built with ``CONFIG_INPUT_KEYBOARD_BYTESTREAM``, in which case it
delivers the byte stream that the keyboard codec defines. This follows
that setting rather than having a switch of its own, and says which one it
is on the first line.

Injecting keys
--------------

With ``-i`` the tool goes the other way and writes into a virtual keyboard,
so that an application can be driven with no keyboard hardware at all.
This needs ``CONFIG_UINPUT_KEYBOARD``, which registers ``/dev/ukeyboard``.

Given no source it injects what it reads from its own stdin, which is the
serial console::

    nsh> kbd -i /dev/ukeyboard
    kbd: injecting into /dev/ukeyboard, type to send, Ctrl-D to stop

Given a source it forwards every key of one keyboard onto another::

    nsh> kbd -i /dev/ukeyboard /dev/kbda &
    kbd: forwarding /dev/kbda into /dev/ukeyboard

Point an application at ``/dev/ukeyboard`` and both reach it at once,
which the application cannot do by itself since it opens a single device.

.. note::

   Raise ``CONFIG_UINPUT_KEYBOARD_BUFNUMBER`` before injecting. It counts
   events rather than keys, so the default of eight holds four keystrokes,
   and a console hands over a whole line at once. The keyboard upper half
   overwrites the oldest event when the buffer is full, so a typed line
   arrives with its beginning missing and nothing says so.

Configuration
-------------

- ``CONFIG_SYSTEM_KBD`` enables the application.
- ``CONFIG_SYSTEM_KBD_DEVPATH`` is the keyboard to read when none is named
  on the command line. Default ``/dev/kbd0``. Note that the USB HID driver
  names its devices ``/dev/kbda`` onwards, since they come and go as
  keyboards are plugged in.
- ``CONFIG_SYSTEM_KBD_PROGNAME``, ``CONFIG_SYSTEM_KBD_PRIORITY`` and
  ``CONFIG_SYSTEM_KBD_STACKSIZE`` are the usual task settings.
