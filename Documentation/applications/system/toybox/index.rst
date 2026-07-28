===================================
``toybox`` Toybox Command Suite
===================================

Overview
========

Toybox (https://landley.net/toybox/) is a 0BSD-licensed multi-call binary
providing a POSIX/LSB command suite and a small interactive shell. The
``toybox`` application downloads a pinned upstream release during the
build, patches it for NuttX, and builds it as a single NuttX task
(``PROGNAME=toybox``).

Run with no arguments it acts as an interactive ``toybox>`` command
prompt, suitable for use as the system's ``CONFIG_INIT_ENTRYPOINT``. Run
with arguments (``toybox ls``, ``toybox cat file``) it runs one command
and exits, which is how it is invoked as an NSH builtin.

Toybox does not depend on NSH in either direction. When both are enabled,
each can invoke the other at runtime: NSH via its own
``CONFIG_NSH_BUILTIN_APPS``, Toybox via ``CONFIG_SYSTEM_TOYBOX_BUILTIN_BRIDGE``.

Configuration
=============

Enable the application with ``CONFIG_SYSTEM_TOYBOX``, under
:menuselection:`System Libraries and NSH Add-Ons` in ``menuconfig``.

- ``CONFIG_SYSTEM_TOYBOX_VERSION`` -- upstream release tag to download
- ``CONFIG_SYSTEM_TOYBOX_PRIORITY``, ``CONFIG_SYSTEM_TOYBOX_STACKSIZE``
- ``CONFIG_SYSTEM_TOYBOX_BUILTIN_BRIDGE`` -- run other NuttX builtin
  applications (``nsh``, ``hello``, ...) from the Toybox prompt

Which commands are built in is selected individually under the "Toybox
commands" submenu (one ``CONFIG_SYSTEM_TOYBOX_CMD_<NAME>`` option per
applet).

Usage
=====

.. code-block:: console

   nsh> toybox ls /
   nsh> toybox

The second form starts the interactive prompt; ``exit`` or ``quit``
leaves it.

Known limitations
=================

- ``ps`` builds but lists no processes: it parses Linux's
  ``/proc/<pid>/stat``, which has no equivalent in NuttX's procfs.
- ``grep -r`` is unreliable against procfs.
