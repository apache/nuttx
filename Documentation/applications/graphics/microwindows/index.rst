.. _applications_graphics_microwindows:

Microwindows
============

Microwindows is a lightweight windowing system for small- to medium-footprint
embedded systems.  It provides:

- **GDI / Win32 API** -- a low-level C graphics library for direct framebuffer
  drawing, suitable for tiny single-threaded applications.
- **mwin** -- a widget toolkit built on the Win32 API, supporting windows,
  menus, buttons, edit boxes, list boxes, scroll bars, progress bars, and
  timers.
- **Nano-X / X11 API** -- an X11-compatible client/server window system (not
  yet enabled for NuttX; work in progress).

Currently the NuttX integration only enables the GDI / Win32 API and the
mwin widget toolkit.  The Nano-X server and X11 client libraries are part of
the upstream project but are not yet wired into the NuttX build.

Quick Start
===========

Microwindows requires a framebuffer device (``CONFIG_VIDEO_FB=y``).

QEMU x86-64
-----------

The ``qemu-intel64:mw`` configuration uses the Multiboot2 framebuffer,
a USB HID keyboard (raw byte stream), and a USB HID mouse::

   cd nuttx
   tools/configure.sh qemu-intel64:mw
   make -j$(nproc)

The build produces a flat ``nuttx.elf`` image.  To run it on QEMU you need a
bootable ISO with GRUB2.  See :doc:`/platforms/x86_64/intel64/index` for the
step-by-step procedure.  In short:

#. Create the ISO directory structure::

      mkdir -p iso/boot/grub

#. Write ``grub.cfg``::

      echo 'set timeout=0'          > iso/boot/grub/grub.cfg
      echo 'set default=0'         >> iso/boot/grub/grub.cfg
      echo 'menuentry "kernel" {'  >> iso/boot/grub/grub.cfg
      echo '  multiboot2 /boot/nuttx.elf' >> iso/boot/grub/grub.cfg
      echo '}'                     >> iso/boot/grub/grub.cfg

#. Copy ``nuttx.elf`` and create the ISO::

      cp nuttx.elf iso/boot/
      grub-mkrescue -o boot.iso iso

#. Run QEMU (omit ``-nographic`` so the framebuffer window opens)::

      qemu-system-x86_64 -cpu host -enable-kvm -m 2G -cdrom boot.iso

The pre-built config sets ``CONFIG_INIT_ENTRYPOINT="mwdemo_main"`` so the
demo starts immediately.

Simulator
---------

The ``sim:mw`` configuration uses the virtual keyboard, touchscreen, and
X11-based framebuffer::

   cd nuttx
   tools/configure.sh sim:mw
   make -j$(nproc)

This builds a host-native ``./nuttx`` binary that opens an X11 window.
Keyboard input goes to the window that has focus; touch events are generated
by mouse clicks within the window.

See also the respective board documentation for configuration details:

* :doc:`/platforms/sim/sim/boards/sim/index` — ``sim:mw`` configuration
* :doc:`/platforms/x86_64/intel64/boards/qemu-intel64/index` — ``qemu-intel64:mw`` configuration

Available Configurations
========================

To enable Microwindows, set ``CONFIG_GRAPHICS_MICROWINDOWS=y``.
The library is located at ``apps/graphics/microwindows``.

Dependencies
------------

All configurations require ``CONFIG_VIDEO_FB=y``.

For **qemu-intel64:mw** the following input-related options are required:

  ``CONFIG_USBHOST=y``, ``CONFIG_USBHOST_HIDKBD=y``, ``CONFIG_USBHOST_HIDMOUSE=y``,
  ``CONFIG_USBHOST_XHCI_PCI=y``, ``CONFIG_USBHOST_WAITER=y``,
  ``CONFIG_LIBC_KBDCODEC=y``, ``CONFIG_HIDKBD_ENCODED=y``

For **sim:mw** the following input options are required:

  ``CONFIG_INPUT=y``, ``CONFIG_SIM_KEYBOARD=y``, ``CONFIG_SIM_TOUCHSCREEN=y``

Keyboard Drivers
----------------

``choice`` prompt: ``"Keyboard driver"``

``MICROWINDOWS_KBD_EVENT``
    Reads ``keyboard_event_s`` events from ``/dev/kbd`` (path configurable).
    Supports key press/release tracking and modifier keys (Shift, Ctrl, Alt).

    Device path config: ``MICROWINDOWS_KBD_EVENT_PATH`` (default ``/dev/kbd``).

``MICROWINDOWS_KBD_RAW``
    Reads raw byte stream from ``/dev/kbda`` (path configurable).  Decodes
    escape sequences via the ``kbd_codec`` library.  Requires
    ``LIBC_KBDCODEC=y``.  Suitable for USB HID keyboards.

    Device path config: ``MICROWINDOWS_KBD_RAW_PATH`` (default ``/dev/kbda``).

``MICROWINDOWS_KBD_NONE``
    No keyboard driver is compiled.  Microwindows uses a null driver that
    never produces input.  Suitable for display-only applications.

``MICROWINDOWS_KBD_CUSTOM``
    No built-in keyboard driver is compiled.  The application or BSP must
    provide its own ``kbddev`` symbol and ``KBDDEVICE`` implementation, and
    add the source file to the build (e.g. via ``CSRCS`` in its Makefile).

Mouse / Touchscreen Drivers
---------------------------

``choice`` prompt: ``"Mouse/touchscreen driver"``

``MICROWINDOWS_MOUSE_RELATIVE``
    Reads ``mouse_report_s`` events from ``/dev/mouse0`` (path configurable).
    Supports three buttons and scroll wheel.

    Device path config: ``MICROWINDOWS_MOUSE_PATH`` (default ``/dev/mouse0``).

``MICROWINDOWS_MOUSE_TS``
    Reads ``touch_sample_s`` events from ``/dev/input0`` (path configurable).
    Supports absolute positioning.

    Device path config: ``MICROWINDOWS_TS_PATH`` (default ``/dev/input0``).

``MICROWINDOWS_MOUSE_NONE``
    No pointing device driver is compiled.  Microwindows uses a null driver
    that hides the cursor and produces no input.

``MICROWINDOWS_MOUSE_CUSTOM``
    No built-in pointing device driver is compiled.  The application or BSP
    must provide its own ``mousedev`` symbol and ``MOUSEDEVICE``
    implementation, and add the source file to the build.

Framebuffer
-----------

``MICROWINDOWS_FB_PATH``
    Path to the NuttX framebuffer device.  Default ``/dev/fb0``.

Microwindows Demo Application
=============================

Enable with ``CONFIG_EXAMPLES_MICROWINDOWS=y``.  The example is located at
``apps/examples/microwindows``.

It ports ``mwdemo.c`` from upstream, which exercises the mwin widget toolkit
(buttons, edit boxes, list boxes, scroll bars, progress bars).  The demo uses
``GdOpenScreen()`` to initialise the framebuffer and enters a Win32-style
message loop with ``GetMessage()`` / ``DispatchMessage()``.

.. figure:: mwdemo-sim.png
   :align: center
   :alt: The Microwindows mwdemo running on the NuttX simulator.

   The mwdemo application running on the NuttX simulator (``sim:mw``).

``EXAMPLES_MICROWINDOWS_PROGNAME``
    Program name for the NSH ELF install (default ``"mwexample"``).
``EXAMPLES_MICROWINDOWS_PRIORITY``
    Task priority (default ``100``).  For setups where input threads
    have a lower priority, consider lowering this value.

    .. warning::

       On ``qemu-intel64:mw`` the default USB HID mouse thread priority
       (``CONFIG_HIDMOUSE_DEFPRIO``, default 50) is lower than the demo
       application priority (100).  When dragging a window, Microwindows'
       ``MwSelect`` switches to polling (``select`` with timeout 0), which can
       starve the USB input thread and make the whole system appear frozen.
       Raise ``CONFIG_HIDMOUSE_DEFPRIO`` (e.g. to 120), lower
       ``EXAMPLES_MICROWINDOWS_PRIORITY``, or fix the drag loop to use a small
       non-zero timeout.

``EXAMPLES_MICROWINDOWS_STACKSIZE``
    Stack size in bytes (default ``32768``).

Porting to New Hardware
=======================

The Microwindows source is downloaded at build time from the upstream
repository.  NuttX-specific screen, keyboard, and mouse drivers live
in upstream ``src/drivers/`` (``scr_nuttx.c``, ``kbd_nuttx_event.c``,
``kbd_nuttx_raw.c``, ``mou_nuttx_mouse.c``, ``mou_nuttx_ts.c``).

A board with a working framebuffer and input devices needs:

#. A ``defconfig`` enabling ``CONFIG_VIDEO_FB``, ``GRAPHICS_MICROWINDOWS``,
   and the appropriate keyboard/mouse driver choice.
#. For framebuffer-only (display-only) use, select ``KBD_NONE`` and
   ``MOUSE_NONE``.

If the board's input hardware does not fit the built-in drivers, select
``KBD_CUSTOM`` and/or ``MOUSE_CUSTOM`` and provide your own driver source.

Resources
=========

- `Microwindows upstream repository <https://github.com/ghaerr/microwindows>`_
- `NuttX Discussion Issue #18566 <https://github.com/apache/nuttx/issues/18566>`_
