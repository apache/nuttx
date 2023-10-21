``nxtext`` Display NX Text
==========================

This directory contains another simple test of a subset of the NX APIs defined
in ``include/nuttx/nx/nx.h``. This text focuses on text displays on the display
background combined with pop-up displays over the text. The text display will
continue to update while the pop-up is visible.

**Note**: This example will **only** work with FB drivers and with LCD drivers
that support reading the contents of the internal LCD memory **unless** you
define ``CONFIG_EXAMPLES_NXTEXT_NOGETRUN``. If you notice garbage on the display
or a failure at the point where the display should scroll, it is probably
because you have an LCD driver that is write-only.

The following configuration options can be selected:

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the ``NXTEXT`` example as a built-in that
  can be executed from the NSH command line.
- ``CONFIG_EXAMPLES_NXTEXT_BGCOLOR`` – The color of the background. Default
  depends on ``CONFIG_EXAMPLES_NXTEXT_BPP``.
- ``CONFIG_EXAMPLES_NXTEXT_BGFONTID`` – Selects the font to use in the background
  text (see font ID numbers in ``include/nuttx/nx/nxfonts.h``).
- ``CONFIG_EXAMPLES_NXTEXT_BGFONTCOLOR`` – The color of the fonts used in the
  background window. Default depends on ``CONFIG_EXAMPLES_NXTEXT_BPP``.
- ``CONFIG_EXAMPLES_NXTEXT_PUCOLOR`` – The color of the pop-up window. Default
  depends on ``CONFIG_EXAMPLES_NXTEXT_BPP``.
- ``CONFIG_EXAMPLES_NXTEXT_PUFONTID`` – Selects the font to use in the pop-up
  windows (see font ID numbers in ``include/nuttx/nx/nxfonts.h``).
- ``CONFIG_EXAMPLES_NXTEXT_PUFONTCOLOR`` – The color of the fonts used in the
  background window. Default depends on ``CONFIG_EXAMPLES_NXTEXT_BPP``.
- ``CONFIG_EXAMPLES_NXTEXT_BPP`` – Pixels per pixel to use. Valid options include
  ``2``, ``4``, ``8``, ``16``, ``24`` and ``32``. Default is ``32``.
- ``CONFIG_EXAMPLES_NXTEXT_NOGETRUN`` – If your display is read-only OR if reading
  is not reliable, then select this configuration to avoid reading from the
  display.
- ``CONFIG_EXAMPLES_NXTEXT_BMCACHE`` – The maximum number of characters that can
  be put in the background window. Default is ``128``.
- ``CONFIG_EXAMPLES_NXTEXT_GLCACHE`` – The maximum number of pre-rendered fonts
  that can be retained for the background window.
- ``CONFIG_EXAMPLES_NXTEXT_STACKSIZE`` – The stacksize to use when creating the NX
  server. Default ``2048``.
- ``CONFIG_EXAMPLES_NXTEXT_CLIENTPRIO`` – The client priority. Default: ``100``.
- ``CONFIG_EXAMPLES_NXTEXT_SERVERPRIO`` – The server priority. Default: ``120``.
- ``CONFIG_EXAMPLES_NXTEXT_LISTENERPRIO`` – The priority of the event listener
  thread. Default: ``80``.
- ``CONFIG_EXAMPLES_NXTEXT_NOTIFYSIGNO`` – The signal number to use with
  ``nx_eventnotify()``. Default: ``32``.

The example also expects the following settings and will generate an error if
they are not as expected::

  CONFIG_DISABLE_MQUEUE=n
  CONFIG_DISABLE_PTHREAD=n
  CONFIG_NX_BLOCKING=y
