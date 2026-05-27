===============================
``hostname`` "hostname" command
===============================

Overview
========

The ``hostname`` command displays or sets the system hostname. When called
without arguments, it prints the current hostname. When called with a
hostname argument, it sets the system hostname to the specified value.

The hostname is stored in the kernel and can be retrieved by applications
using the ``gethostname()`` system call. It is typically used to identify
the system on a network and in shell prompts.

Configuration
=============

Enable the command with ``CONFIG_SYSTEM_HOSTNAME``. This option has no
additional dependencies.

Usage
=====

.. code-block:: console

   hostname

Display the current system hostname.

.. code-block:: console

   hostname <hostname>

Set the system hostname to ``<hostname>``.

.. code-block:: console

   hostname -F <file>

Read the hostname from the specified file and set it as the system hostname.

Options
=======

``-F <file>``
  Read the hostname from the specified file. The first line of the file
  is used as the hostname. Trailing newlines are stripped.

``-h``
  Display usage information and exit.

Examples
========

Display the current hostname:

.. code-block:: console

   nsh> hostname
   nuttx

Set a new hostname:

.. code-block:: console

   nsh> hostname mydevice
   nsh> hostname
   mydevice

Read hostname from a file:

.. code-block:: console

   nsh> cat /etc/hostname
   embedded-device
   nsh> hostname -F /etc/hostname
   nsh> hostname
   embedded-device

Display usage information:

.. code-block:: console

   nsh> hostname -h
   Usage: hostname [<hostname>|-F <file>]

Notes
=====

- The hostname must be between 1 and ``HOST_NAME_MAX`` characters long.
- Setting an empty hostname or a hostname longer than ``HOST_NAME_MAX``
  will result in an error.
- The hostname is stored in the kernel and persists until the system is
  rebooted or the hostname is changed again.
- The ``-F`` option reads only the first line of the specified file and
  strips any trailing newline characters.
- If both a hostname argument and the ``-F`` option are provided, the
  ``-F`` option takes precedence.
