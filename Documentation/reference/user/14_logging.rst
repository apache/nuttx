=======
Logging
=======

NuttX provides the SYSLOG for application and OS logging, which can be
configured in various ways to select how these messages are displayed
(see details :doc:`here </components/drivers/special/syslog>`).

Applications can emit logging messages using the standard :c:func:`syslog`
interface.

.. note:: The standard :c:func:`openlog` and :c:func:`closelog`
  are not currently supported.

.. c:function:: void syslog(int priority, const char *fmt, ...)

  This interface allows to send messages to SYSLOG using standard
  :c:func:`printf` formatting. 

  Each message sent to SYSLOG is assigned a priority. Depending
  on system configuration this message may or not appear in the
  output.

  :param priority: A priority given by ``LOG_*`` family of
    definitions.

  :param fmt: The format string

.. c:function:: void vsyslog(int priority, const char *fmt, va_list ap)

  Performs the same task as :c:func:`syslog` with the
  difference that it takes a set of arguments which have been obtained
  using the :file:`include/stdarg.h` variable argument list macros.

.. c:function:: int setlogmask(int mask)

  Sets the logging mask which controls which messages appear on SYSLOG
  output. :c:func:`setlogmask` is not a thread-safe, re-entrant function.
  Concurrent use will have undefined behavior.
  
  :param mask: The new mask to set.
    See :c:macro:`LOG_MASK` and :c:macro:`LOG_UPTO`.
    Per OpenGroup.org "If the maskpri argument is 0, the current log mask
    is not modified."  In this implementation, the value zero is permitted
    in order to disable all syslog levels.
  
  :returns: The previous mask. 
    
  .. warning:: Per POSIX the syslog mask should be a per-process value but in
    NuttX, the scope of the mask is dependent on the nature of the build:

    * Flat Build:  There is one, global SYSLOG mask that controls all output.
    * Protected Build:  There are two SYSLOG masks.  One within the kernel
      that controls only kernel output.  And one in user-space that controls
      only user SYSLOG output.
    * Kernel Build:  The kernel build is compliant with the POSIX requirement:
      There will be one mask for each user process, controlling the SYSLOG
      output only form that process.  There will be a separate mask
      accessible only in the kernel code to control kernel SYSLOG output.

Priority Levels
===============

The following levels are defined:

================ ===========
Priority (macro) Description
================ ===========
``LOG_EMERG``    System is unusable
``LOG_ALERT``    Action must be taken immediately
``LOG_CRIT``     Critical conditions
``LOG_ERR``      Error conditions
``LOG_WARNING``  Warning conditions
``LOG_NOTICE``   Normal, but significant, condition
``LOG_INFO``     Informational message
``LOG_DEBUG``    Debug-level message
================ ===========

Priority mask
=============

The following macros can be used with :c:func:`setlogmask`:

.. c:macro:: LOG_MASK(p)

  Returns the logmask corresponding priority ``p``

.. c:macro:: LOG_UPTO(p)

  Returns the logmask of all SYSLOG priorities
  up to and including ``p``.

.. c:macro:: LOG_ALL

  Mask corresponding to all priorities enabled