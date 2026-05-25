===================================
``setlogmask`` "setlogmask" command
===================================

Overview
========

The ``setlogmask`` command changes which syslog priority levels are
accepted by the NuttX syslog mask.  It is useful when debugging because it
can raise the mask to include verbose messages or lower it so that only
high-priority messages are emitted.

The command calls ``setlogmask(LOG_UPTO(<priority>))``.  A selected priority
therefore enables messages at that priority and all higher-priority syslog
levels.

Configuration
=============

Enable the application with ``CONFIG_SYSTEM_SETLOGMASK``.

The program name, task priority, and stack size are controlled by:

- ``CONFIG_SYSTEM_SETLOGMASK_PROGNAME``
- ``CONFIG_SYSTEM_SETLOGMASK_PRIORITY``
- ``CONFIG_SYSTEM_SETLOGMASK_STACKSIZE``

When ``CONFIG_SYSLOG_IOCTL`` is enabled, the command also supports listing
syslog channels and enabling or disabling a named channel through
``/dev/log`` ioctls.

Usage
=====

.. code-block:: console

   setlogmask <d|i|n|w|e|c|a|r>
   setlogmask list
   setlogmask <enable|disable> <channel>
   setlogmask -h

The ``list``, ``enable``, and ``disable`` forms are available only when
``CONFIG_SYSLOG_IOCTL`` is enabled.

Priority arguments
==================

Each single-letter argument selects the lowest priority level that remains
enabled by the mask:

.. list-table::
   :header-rows: 1

   * - Argument
     - Syslog priority
   * - ``d``
     - ``LOG_DEBUG``
   * - ``i``
     - ``LOG_INFO``
   * - ``n``
     - ``LOG_NOTICE``
   * - ``w``
     - ``LOG_WARNING``
   * - ``e``
     - ``LOG_ERR``
   * - ``c``
     - ``LOG_CRIT``
   * - ``a``
     - ``LOG_ALERT``
   * - ``r``
     - ``LOG_EMERG``

Examples
========

Allow messages through ``LOG_DEBUG``:

.. code-block:: console

   nsh> setlogmask d

Limit syslog output to emergency, alert, critical, error, and warning
messages:

.. code-block:: console

   nsh> setlogmask w

List configured syslog channels when channel ioctls are enabled:

.. code-block:: console

   nsh> setlogmask list
   Channels:
     syslog: enable

Disable a named syslog channel when channel ioctls are enabled:

.. code-block:: console

   nsh> setlogmask disable syslog

Notes
=====

The scope of the syslog mask depends on the NuttX build configuration.  In a
flat build there is one global mask.  In a protected build, kernel and user
space each have a separate mask.  In a kernel build, the user-space mask is
per process and kernel code has a separate mask.
