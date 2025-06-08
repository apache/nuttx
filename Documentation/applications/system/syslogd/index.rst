===========
``syslogd``
===========

.. tags:: experimental

The ``syslogd`` command is used as a "syslog daemon". It sends syslog entries
over the network using UDP so that they can be consumed by a central logging
server. This is achieved by reading the newline separated long entries from the
``syslog`` device specified by ``CONFIG_SYSLOG_DEVPATH`` (similar to how
``dmesg`` works) and sending them as UDP packets to the configured host.

If the ``syslog`` device being read is non-blocking, ``syslogd`` will terminate
with a successful error code once all logs have been read. It is suggested to
use a blocking device so that ``syslogd`` will continuously send logs when they
are generated.

In order to use this command, you must have a UDP capable platform and have
enabled `RFC 5424 <https://www.rfc-editor.org/rfc/rfc5424>`_ compatible syslog
messages: ``CONFIG_SYSLOG_RFC5424``.

Options are similar to the Linux `syslogd
<https://linux.die.net/man/8/syslogd>`_ utility, however not all options are
implemented since NuttX does not immediately support the more complex features
(i.e. the ``-l`` option). The application can only act as an originator, it does
not receive or forward logs.

.. warning::

   The daemon is currently runs itself in the background using ``fork()``. On
   architectures where ``fork()`` is not implemented, the daemon must be
   "backgrounded" by using the trailing ``&`` in NSH at the moment. It is also
   possible to us ``posix_spawn()`` from a parent program.

Read more about ``syslog`` on NuttX: :doc:`/components/drivers/special/syslog`
