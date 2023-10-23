==========================
``nx`` NX graphics example
==========================

This directory contains a simple test of a subset of the NX APIs defined in
``include/nuttx/nx/nx.h``. The following configuration options can be selected:

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the NX example as a built-in that can be
  executed from the NSH command line
- ``CONFIG_EXAMPLES_NX_BGCOLOR`` – The color of the background. Default depends on
  ``CONFIG_EXAMPLES_NX_BPP``.
- ``CONFIG_EXAMPLES_NX_COLOR1`` – The color of window 1. Default depends on
  ``CONFIG_EXAMPLES_NX_BPP``.
- ``CONFIG_EXAMPLES_NX_COLOR2`` – The color of window 2. Default depends on
  ``CONFIG_EXAMPLES_NX_BPP``.
- ``CONFIG_EXAMPLES_NX_TBCOLOR`` – The color of the toolbar. Default depends on
  ``CONFIG_EXAMPLES_NX_BPP``.
- ``CONFIG_EXAMPLES_NX_FONTID`` – Selects the font (see font ID numbers in
  ``include/nuttx/nx/nxfonts.h``).
- ``CONFIG_EXAMPLES_NX_FONTCOLOR`` – The color of the fonts. Default depends on
  ``CONFIG_EXAMPLES_NX_BPP``.
- ``CONFIG_EXAMPLES_NX_BPP`` – Pixels per pixel to use. Valid options include ``2``,
  ``4``, ``8``, ``16``, ``24`` and ``32``. Default is ``32``.
- ``CONFIG_EXAMPLES_NX_RAWWINDOWS`` – Use raw windows; Default is to use pretty,
  framed NXTK windows with toolbars.
- ``CONFIG_EXAMPLES_NX_STACKSIZE`` – The stacksize to use when creating the NX
  server. Default ``2048``.
- ``CONFIG_EXAMPLES_NX_CLIENTPRIO`` – The client priority. Default: ``100``
- ``CONFIG_EXAMPLES_NX_SERVERPRIO`` – The server priority. Default: ``120``
- ``CONFIG_EXAMPLES_NX_LISTENERPRIO`` – The priority of the event listener thread.
  Default ``80``.

The example also has the following settings and will generate an error if they
are not as expected::

  CONFIG_DISABLE_MQUEUE=n
  CONFIG_DISABLE_PTHREAD=n
  CONFIG_NX_BLOCKING=y
  CONFIG_BOARDCTL=y
