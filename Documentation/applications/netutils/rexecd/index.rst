==================================
``rexecd`` Remote Execution Server
==================================

``rexecd`` is the daemon that serves remote execution requests from the
``rexec`` client. For the IP families it listens on the rexec port
(``REXECD_PORT``, 512); for ``AF_RPMSG`` it listens on a corresponding
RPMsg socket name. It authenticates the request and runs the requested
command, piping its output back to the client.

By default ``rexecd`` spawns a detached worker thread per accepted
connection so that multiple sessions can run concurrently.

Usage
=====

.. code-block:: none

   rexecd [-4|-6|-r] [-t]

Options
=======

``-4``
  Specify the address family as ``AF_INET`` (IPv4). This is the default.

``-6``
  Specify the address family as ``AF_INET6`` (IPv6).

``-r``
  Specify the address family as ``AF_RPMSG``. This serves the daemon over
  an RPMsg link instead of TCP/IP, allowing a remote processor in an AMP
  system to execute commands. See :doc:`/components/drivers/special/rpmsg/concepts`
  for details on RPMsg.

``-t``
  Serve each connection inline in the main task instead of spawning a
  per-connection worker thread. This avoids allocating the worker stack
  (``CONFIG_NETUTILS_REXECD_STACKSIZE``) from the heap, which helps on
  low-memory targets. With this option connections are served strictly
  one at a time: a long-running or interactive command blocks the accept
  loop until it finishes.
