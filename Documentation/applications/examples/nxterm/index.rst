``nxterm`` Display NuttShell (NSH) as NX Console
================================================

This directory contains yet another version of the NuttShell (NSH). This version
uses the NX console device defined in ``include/nuttx/nx/nxterm.h`` for output.
the result is that the NSH input still come from the standard console input
(probably a serial console). But the text output will go to an NX winbdow.
Prerequisite configuration settings for this test include:

- ``CONFIG_NX=y`` – NX graphics must be enabled
- ``CONFIG_NXTERM=y`` – The NX console driver must be built
- ``CONFIG_DISABLE_MQUEUE=n`` – Message queue support must be available.
- ``CONFIG_DISABLE_PTHREAD=n`` – pthreads are needed
- ``CONFIG_NX_BLOCKING=y`` – pthread APIs must be blocking
- ``CONFIG_NSH_CONSOLE=y`` – NSH must be configured to use a console.

The following configuration options can be selected to customize the test:

- ``CONFIG_EXAMPLES_NXTERM_BGCOLOR`` – The color of the background. Default
  Default is a darker royal blue.
- ``CONFIG_EXAMPLES_NXTERM_WCOLOR`` – The color of the window. Default is a light
  slate blue.
- ``CONFIG_EXAMPLES_NXTERM_FONTID`` – Selects the font (see font ID numbers in
  ``include/nuttx/nx/nxfonts.h``).
- ``CONFIG_EXAMPLES_NXTERM_FONTCOLOR`` – The color of the fonts. Default is black.
- ``CONFIG_EXAMPLES_NXTERM_BPP`` – Pixels per pixel to use. Valid options include
  ``2``, ``4``, ``8``, ``16``, ``24`` and ``32``. Default is ``32``.
- ``CONFIG_EXAMPLES_NXTERM_TOOLBAR_HEIGHT`` – The height of the toolbar. Default:
  ``16``.
- ``CONFIG_EXAMPLES_NXTERM_TBCOLOR`` – The color of the toolbar. Default is a
  medium grey.
- ``CONFIG_EXAMPLES_NXTERM_MINOR`` – The NX console device minor number. Default
  is ``0`` corresponding to ``/dev/nxterm0``.
- ``CONFIG_EXAMPLES_NXTERM_DEVNAME`` – The quoted, full path to the NX console
  device corresponding to ``CONFIG_EXAMPLES_NXTERM_MINOR``. Default:
  ``/dev/nxterm0``.
- ``CONFIG_EXAMPLES_NXTERM_PRIO`` – Priority of the NxTerm task. Default:
  ``SCHED_PRIORITY_DEFAULT``.
- ``CONFIG_EXAMPLES_NXTERM_STACKSIZE`` – Stack size allocated for the NxTerm task.
  Default: ``2048``.
- ``CONFIG_EXAMPLES_NXTERM_STACKSIZE`` – The stacksize to use when creating the NX
  server. Default: ``2048``.
- ``CONFIG_EXAMPLES_NXTERM_CLIENTPRIO`` – The client priority. Default: ``100``.
- ``CONFIG_EXAMPLES_NXTERM_SERVERPRIO`` – The server priority. Default: ``120``.
- ``CONFIG_EXAMPLES_NXTERM_LISTENERPRIO`` – The priority of the event listener
  thread. Default: ``80``.
