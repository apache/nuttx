.. _syslog:

======
SysLog
======

Standard SysLog Interfaces
==========================

The NuttX SYSLOG is an architecture for getting debug and status information
from the system. The syslogging interfaces are defined in the header file
``include/syslog.h``.

The primary interface to SYSLOG sub-system is the function ``syslog()`` and,
to a lesser extent, its companion ``vsyslog()``:

.. code-block:: c

  /****************************************************************************
   * Name: syslog and vsyslog
   *
   * Description:
   *   syslog() generates a log message. The priority argument is formed by
   *   ORing the facility and the level values (see include/syslog.h). The
   *   remaining arguments are a format, as in printf and any arguments to the
   *   format.
   *
   *   The NuttX implementation does not support any special formatting
   *   characters beyond those supported by printf.
   *
   *   The function vsyslog() performs the same task as syslog() with the
   *   difference that it takes a set of arguments which have been obtained
   *   using the stdarg variable argument list macros.
   *
   ****************************************************************************/
  
  int syslog(int priority, FAR const IPTR char *format, ...);
  int vsyslog(int priority, FAR const IPTR char *src, va_list ap);


The additional ``setlogmask()`` interface can use use to filter
SYSLOG output:

.. code-block:: c

  /****************************************************************************
   * Name: setlogmask
   *
   * Description:
   *   The setlogmask() function sets the logmask and returns the previous
   *   mask. If the mask argument is 0, the current logmask is not modified.
   *
   *   The SYSLOG priorities are: LOG_EMERG, LOG_ALERT, LOG_CRIT, LOG_ERR,
   *   LOG_WARNING, LOG_NOTICE, LOG_INFO, and LOG_DEBUG.  The bit corresponding
   *   to a priority p is LOG_MASK(p); LOG_UPTO(p) provides the mask of all
   *   priorities in the above list up to and including p.
   *
   *   Per OpenGroup.org "If the maskpri argument is 0, the current log mask
   *   is not modified."  In this implementation, the value zero is permitted
   *   in order to disable all syslog levels.
   *
   *   REVISIT: Per POSIX the syslog mask should be a per-process value but in
   *   NuttX, the scope of the mask is dependent on the nature of the build:
   *
   *   Flat Build:  There is one, global SYSLOG mask that controls all output.
   *   Protected Build:  There are two SYSLOG masks.  One within the kernel
   *     that controls only kernel output.  And one in user-space that controls
   *     only user SYSLOG output.
   *   Kernel Build:  The kernel build is compliant with the POSIX requirement:
   *     There will be one mask for for each user process, controlling the
   *     SYSLOG output only form that process.  There will be a separate mask
   *     accessable only in the kernel code to control kernel SYSLOG output.
   *
   ****************************************************************************/
  
  int setlogmask(int mask);


These are all standard interfaces as defined at https://www.OpenGroup.org.


Debug Interfaces
================

**In NuttX syslog output is really synonymous to debug output** and,
therefore, the debugging interface macros defined in the header file
``include/debug.h`` are also syslogging interfaces.
Those macros are simply wrappers around ``syslog()``.

.. note:: Debug here means "system log" rather than on-chip-debug.

The debugging interfaces differ from the syslog interfaces in that:

* They do not take a priority parameter; the priority is inherent
  in the debug macro name.
* They decorate the output stream with information such as the file name.
* They can each be disabled via configuration options.

Each debug macro has a base name that represents the priority and a prefix
that represents the sub-system.
Each macro is individually initialized by both priority and sub-system.
For example, ``uerr()`` is the macro used for error level messages
from the USB subsystem and is enabled with ``CONFIG_DEBUG_USB_ERROR``.

The base debug macro names, their priority, and configuration variable
are summarized below:

* ``info()``: The ``info()`` macro is the lowest priority (``LOG_INFO``)
  and is intended to provide general information about the flow of program
  execution so that you can get an overview of the behavior of the program.
  ``info()`` is often very chatty and voluminous and usually more information
  than you may want to see. The ``info()`` macro is controlled via
  ``CONFIG_DEBUG+subsystem+INFO``.
* ``warn()``: The ``warn()`` macro has medium priority (``LOG_WARN``)
  and is controlled by ``CONFIG_DEBUG+subsystem+WARN``. The ``warn()`` is
  intended to note exceptional or unexpected conditions that might be
  potential errors or, perhaps, minor errors that easily recovered.
* ``err()``: This is a high priority debug macro (``LOG_ERROR``)
  and controlled by ``CONFIG_DEBUG+subsystem+ERROR``.
  The ``err()`` is reserved to indicate important error conditions.
* ``alert()``: The highest priority debug macro (``LOG_EMERG``)
  and is controlled by ``CONFIG_DEBUG_ALERT``. The ``alert()`` macro
  is reserved for use solely by assertion and crash handling logic.
  It also differs from the other macros in that it is global
  and cannot be enabled or disabled per subsystem.


SYSLOG Channels
===============

SYSLOG Channel Interfaces
-------------------------

In the NuttX SYSLOG implementation, the underlying device logic the supports
the SYSLOG output is referred to as a SYSLOG channel.
Each SYSLOG channel is represented by an interface defined
in ``include/nuttx/syslog/syslog.h``:

.. code-block:: c

  /* This structure provides the interface to a SYSLOG device */
  
  typedef CODE int (*syslog_putc_t)(int ch);
  typedef CODE int (*syslog_flush_t)(void);
  
  struct syslog_channel_s
  {
    /* I/O redirection methods */
  
    syslog_putc_t sc_putc;    /* Normal buffered output */
    syslog_putc_t sc_force;   /* Low-level output for interrupt handlers */
    syslog_flush_t sc_flush;  /* Flush buffered output (on crash) */
  
    /* Implementation specific logic may follow */
  };

The channel interface is instantiated by calling ``syslog_channel()``:

.. code-block:: c

  /****************************************************************************
  * Name: syslog_channel
  *
  * Description:
  * Configure the SYSLOGging function to use the provided channel to
  * generate SYSLOG output.
  *
  * Input Parameters:
  * channel - Provides the interface to the channel to be used.
  *
  * Returned Value:
  * Zero (OK)is returned on success. A negated errno value is returned
  * on any failure.
  *
  ****************************************************************************/

  int syslog_channel(FAR const struct syslog_channel_s *channel);

``syslog_channel()`` is a non-standard, internal OS interface
and is not available to applications.
It may be called numerous times as necessary to change channel interfaces.
By default, all system log output goes to console (``/dev/console``).

SYSLOG Channel Initialization
-----------------------------

The initial, default SYSLOG channel is established with statically initialized
global variables so that some level of SYSLOG output may be available
immediately upon reset.
This initialized data is in the file ``drivers/syslog/syslog_channel.c``.

The initial SYSLOG capability is determined by the selected SYSLOG channel:

* In-Memory Buffer (**RAMLOG**). Full SYSLOG capability as available at reset.
* **Serial Console**: If the serial implementation provides the low-level
  character output function ``up_putc()``, then that low level serial output
  is available as soon as the serial device has been configured.
* For all other SYSLOG channels, all SYSLOG output goes to the bit-bucket
  (discarded) until the SYSLOG channel device has been initialized.

The syslog channel device is initialized when the bring-up logic
calls ``syslog_intialize()``:

.. code-block:: c

  /****************************************************************************
   * Name: syslog_initialize
   *
   * Description:
   *   One power up, the SYSLOG facility is non-existent or limited to very
   *   low-level output.  This function is called later in the initialization
   *   sequence after full driver support has been initialized.  It installs
   *   the configured SYSLOG drivers and enables full SYSLOGing capability.
   *
   *   This function performs these basic operations:
   *
   *   - Initialize the SYSLOG device
   *   - Call syslog_channel() to begin using that device.
   *
   *   If CONFIG_ARCH_SYSLOG is selected, then the architecture-specific
   *   logic will provide its own SYSLOG device initialize which must include
   *   as a minimum a call to syslog_channel() to use the device.
   *
   * Input Parameters:
   *   phase - One of {SYSLOG_INIT_EARLY, SYSLOG_INIT_LATE}
   *
   * Returned Value:
   *   Zero (OK) is returned on success; a negated errno value is returned on
   *   any failure.
   *
   ****************************************************************************/
  
  #ifndef CONFIG_ARCH_SYSLOG
  int syslog_initialize(enum syslog_init_e phase);
  #else
  #  define syslog_initialize(phase)
  #endif


Different types of SYSLOG devices have different OS initialization
requirements. Some are available immediately at reset, some are available
after some basic OS initialization, and some only after OS is fully
initialized.
In order to satisfy these different initialization requirements,
``syslog_initialize()`` is called twice from the boot-up logic:

1. ``syslog_initialize()`` is called from the architecture-specific
   ``up_initialize()`` function as some as basic hardware resources
   have been initialized: Timers, interrupts, etc.
   In this case, ``syslog_initialize()`` is called with the argument
   ``SYSLOG_INIT_EARLY``.
2. ``syslog_initialize()`` is called again from ``nx_start()`` when
   the full OS initialization has completed, just before the application
   main entry point is spawned. In this case, ``syslog_initialize()``
   is called with the argument ``SYSLOG_INIT_LATE``.

There are other types of SYSLOG channel devices that may require even further
initialization. For example, the file SYSLOG channel (described below)
cannot be initialized until the necessary file systems have been mounted.

Interrupt Level SYSLOG Output
-----------------------------

As a general statement, SYSLOG output only supports normal output from NuttX
tasks. However, for debugging purposes, it is also useful to get SYSLOG
output from interrupt level logic.
In an embedded system, that is often where the most critical operations
are performed.

There are three conditions under which SYSLOG output generated from interrupt
level processing can a included the SYSLOG output stream:

1. Low-Level Serial Output.
2. In-Memory Buffering.
3. Serialization Buffer.

The SYSLOG interrupt buffer is enabled with ``CONFIG_SYSLOG_INTBUFFER``.
When the interrupt buffer is enabled, you must also provide the size
of the interrupt buffer with ``CONFIG_SYSLOG_INTBUFSIZE``.

Low-Level Serial Output
^^^^^^^^^^^^^^^^^^^^^^^

If you are using a SYSLOG console channel (``CONFIG_SYSLOG_CONSOLE``)
with a serial console (``CONFIG_SYSLOG_SERIAL_CONSOLE``) and if the underlying
architecture supports the low-level ``up_putc()`` interface
(``CONFIG_ARCH_LOWPUTC``), then the SYLOG logic will direct the output
to ``up_putc()`` which is capable of generating the serial output
within the context of an interrupt handler.

There are a few issues in doing this however:

1. ``up_putc()`` is able to generate debug output in any context because
   it disables serial interrupts and polls the hardware directly.
   These polls may take many milliseconds and during that time, all interrupts
   are disable within the interrupt handler. This, of course, interferes with
   the real-time behavior of the RTOS.
2. The output generated by ``up_putc()`` is immediate and in real-time.
   The normal SYSLOG output, on the other hand, is buffered in the serial
   driver and may be delayed with respect to the immediate output by many
   lines. Therefore, the interrupt level SYSLOG ouput provided throug
   ``up_putc()`` is grossly out of synchronization with other debug output.

In-Memory Buffering
^^^^^^^^^^^^^^^^^^^

If the RAMLOG SYSLOG channel is supported, then all SYSLOG output is buffered
in memory. Interrupt level SYSLOG output is no different than normal SYSLOG
output in this case.

Serialization Buffering
^^^^^^^^^^^^^^^^^^^^^^^

A final option is the use the an interrupt buffer to buffer the interrupt
level SYSLOG output. In this case:

1. SYSLOG output generated from interrupt level process in not sent
   to the SYSLOG channel immediately. Rather, it is buffered in the
   interrupt serialization buffer.
2. Later, when the next normal syslog output is generated, it will
   first empty the content of the interrupt buffer to the SYSLOG device
   in the proper context. It will then be followed by the normal syslog
   output. In this case, the interrupt level SYSLOG output will interrupt
   the normal output stream and the interrupt level SYSLOG output will
   be inserted into the correct position in the SYSLOG output when
   the next normal SYLOG output is generated.


SYSLOG Channel Options
======================

SYSLOG Console Device
---------------------

The typical SYSLOG device is the system console.
If you are using a serial console, for example, then the SYSLOG output
will appear on that serial port.

This SYSLOG channel is automatically selected by ``syslog_initialize()``
in the LATE initialization phase based on configuration options.
The configuration options that affect this channel selection include:

* ``CONFIG_DEV_CONSOLE``: This setting indicates that the system supports
  a console device, i.e., that the character device ``/dev/console`` exists.
* ``CONFIG_SERIAL_CONSOLE``: This configuration option is automatically
  selected when a UART or USART is configured as the system console.
  There is no user selection.
* ``CONFIG_SYSLOG_CONSOLE``: This configuration option is manually selected
  from the SYSLOG menu. This is the option that acutally enables the SYSLOG
  console device. It depends on ``CONFIG_DEV_CONSOLE`` and it will
  automatically select ``CONFIG_SYSLOG_SERIAL_CONSOLE`` if
  ``CONFIG_SERIAL_CONSOLE`` is selected.
* ``CONFIG_ARCH_LOWPUTC``: This is an indication from the architecture
  configuration that the platform supports the ``up_putc()`` interface.
  ``up_putc()`` is a very low level UART interface that can even be used
  from interrupt handling.
* ``CONFIG_SYSLOG_SERIAL_CONSOLE``: This enables certain features
  of the SYSLOG operation that depend on a serial console.
  If ``CONFIG_ARCH_LOWPUTC`` is also selected, for example,
  then ``up_putc()`` will be used for the forced SYSLOG output.

Interrupt level SYSLOG output will be lost unless:

1. The interrupt buffer is enabled to support serialization, or
2. A serial console is used and ``up_putc()`` is supported.

.. note::

  The console channel uses the fixed character device at
  ``/dev/console``. The console channel is not synonymous with
  ``stdout`` (or file descriptor ``1``). ``stdout`` is the
  current output from a task when, say, ``printf()`` if used.
  Initially, ``stdout`` does, indeed, use the ``/dev/console``
  device. However, ``stdout`` may subsequently be redirected
  to some other device or file.
  This is always the case, for example, when a transient device
  is used for a console – such as a USB console or a Telnet
  console.
  The SYSLOG channel is not redirected as ``stdout`` is; the SYSLOG
  channel will stayed fixed (unless it is explicitly changed
  via ``syslog_channel()``).

References: ``drivers/syslog/syslog_consolechannel.c`` and
``drivers/syslog/syslog_device.c``.


SYSLOG Character Device
-----------------------

The system console device, ``/dev/console``, is a character driver with
some special properties. However, any character driver may be used as the
SYSLOG output channel. For example, suppose you have a serial console
on ``/dev/ttyS0`` and you want SYSLOG output on ``/dev/ttyS1``.
Or suppose you support only a Telnet console but want to capture
debug output ``/dev/ttyS0``.

This SYSLOG device channel is selected with ``CONFIG_SYSLOG_CHAR`` and has
no other dependencies. Differences fromthe SYSLOG console channel include:

1. ``CONFIG_SYSLOG_DEVPATH``: This configuration option string must be set
   provide the full path to the character device to be used.
2. The forced SYSLOG output always goes to the bit-bucket.
   This means that interrupt level SYSLOG output will be lost unless
   the interrupt buffer is enabled to support serialization.
3. ``CONFIG_SYSLOG_CHAR_CRLF``: If ``CONFIG_SYSLOG_CHAR_CRLF`` is selected,
   then inefeeds in the SYSLOG output will be expanded to
   Carriage Return + Linefeed. Since the character device is not a console
   device, the addition of carriage returns to line feeds would
   not be performed otherwise.
   You would probably want this expansion if you use a serial terminal
   program with the character device output.

References: ``drivers/syslog/syslog_devchannel.c`` and
``drivers/syslog/syslog_device.c``.

SYSLOG File Device
------------------

Files can also be used as the sink for SYSLOG output.
There is, however, a very fundamental difference in using a file as opposed
the system console, a RAM buffer, or character device:
You must first mount the file system that supports the SYSLOG file.
That difference means that the file SYSLOG channel cannot be supported during
the boot-up phase but can be instantiated later when board level logic
configures the application environment, including mounting of the file systems.

The interface ``syslog_file_channel()`` is used to configure
the SYSLOG file channel:

.. code-block:: c

  /****************************************************************************
   * Name: syslog_file_channel
   *
   * Description:
   *   Configure to use a file in a mounted file system at 'devpath' as the
   *   SYSLOG channel.
   *
   *   This tiny function is simply a wrapper around syslog_dev_initialize()
   *   and syslog_channel().  It calls syslog_dev_initialize() to configure
   *   the character file at 'devpath then calls syslog_channel() to use that
   *   device as the SYSLOG output channel.
   *
   *   File SYSLOG channels differ from other SYSLOG channels in that they
   *   cannot be established until after fully booting and mounting the target
   *   file system.  This function would need to be called from board-specific
   *   bring-up logic AFTER mounting the file system containing 'devpath'.
   *
   *   SYSLOG data generated prior to calling syslog_file_channel will, of
   *   course, not be included in the file.
   *
   *   NOTE interrupt level SYSLOG output will be lost in this case unless
   *   the interrupt buffer is used.
   *
   * Input Parameters:
   *   devpath - The full path to the file to be used for SYSLOG output.
   *     This may be an existing file or not.  If the file exists,
   *     syslog_file_channel() will append new SYSLOG data to the end of the
   *     file.  If it does not, then syslog_file_channel() will create the
   *     file.
   *
   * Returned Value:
   *   Zero (OK) is returned on success; a negated errno value is returned on
   *   any failure.
   *
   ****************************************************************************/
  
  #ifdef CONFIG_SYSLOG_FILE
  int syslog_file_channel(FAR const char *devpath);
  #endif


References: ``drivers/syslog/syslog_filechannel.c``,
``drivers/syslog/syslog_device.c``, and ``include/nuttx/syslog/syslog.h``.

SYSLOG RAMLOG Device
--------------------

The RAMLOG is a standalone feature that can be used to buffer any character
data in memory. There are, however, special configurations that can be used
to configure the RAMLOG as a SYSLOG channel.
The RAMLOG functionality is described in a more general way
in the following paragraphs.

RAM Logging Device
^^^^^^^^^^^^^^^^^^

The RAM logging driver is a driver that was intended to support debugging
output (SYSLOG) when the normal serial output is not available.
For example, if you are using a Telnet or USB serial console,
the debug output will get lost – or worse.
For example, what if you want to debug the network over Telnet?

The RAM logging driver can also accept debug output data from interrupt
handler with no special serialization buffering.
As an added benefit, the RAM logging driver is much less invasive.
Since no actual I/O is performed with the debug output is generated,
the RAM logger tends to be much faster and will interfere much less
when used with time critical drivers.

The RAM logging driver is similar to a pipe in that it saves the debugging
output in a circular buffer in RAM.
It differs from a pipe in numerous details as needed to support logging.

This driver is built when CONFIG_RAMLOG is defined in the Nuttx configuration.

dmesg
^^^^^

When the RAMLOG (with SYSLOG) is enabled, a new NuttShell (NSH) command
will appear: ``dmesg``. The dmsg command will dump the contents of the
circular buffer to the console (and also clear the circular buffer).

RAMLOG Configuration options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``CONFIG_RAMLOG`` - Enables the RAM logging feature.
* ``CONFIG_RAMLOG_CONSOLE`` - Use the RAM logging device as a system console.
  If this feature is enabled (along with ``CONFIG_DEV_CONSOLE``),
  then all console output will be re-directed to a circular buffer in RAM.
  This might be useful, for example, if the only console is a Telnet console.
  Then in that case, console output from non-Telnet threads will go to the
  circular buffer and can be viewed using the NSH ``dmesg`` command.
  This optional is not useful in other scenarios.
* ``CONFIG_RAMLOG_SYSLOG`` - Use the RAM logging device for the syslogging
  interface. If this feature is enabled, then all debug output will be
  re-directed to the circular buffer in RAM. This RAM log can be viewed
  from NSH using the ``dmesg`` command.
  NOTE: Unlike the limited, generic character driver SYSLOG device,
  the RAMLOG can be used to capture debug output from interrupt level handlers.
* ``CONFIG_RAMLOG_NPOLLWAITERS`` - The number of threads than can be waiting
  for this driver on ``poll()``. Default: 4.

If ``CONFIG_RAMLOG_CONSOLE`` or ``CONFIG_RAMLOG_SYSLOG`` is selected,
then the following must also be provided:

* ``CONFIG_RAMLOG_BUFSIZE`` - The size of the circular buffer to use.
  Default: 1024 bytes.

Other miscellaneous settings:

* ``CONFIG_RAMLOG_CRLF`` - Pre-pend a carriage return before every linefeed
  that goes into the RAM log.
* ``CONFIG_RAMLOG_NONBLOCKING`` - Reading from the RAMLOG will never block
  if the RAMLOG is empty. If the RAMLOG is empty, then zero is returned
  (usually interpreted as end-of-file). If you do not define this, the NSH
  ``dmsg`` command will lock up when called! So you probably do want this!
* ``CONFIG_RAMLOG_NPOLLWAITERS`` - The maximum number of threads that
  may be waiting on the poll method.


SYSLOG (Input) Character Device
===============================

If the option ``CONFIG_SYSLOG_CHARDEV`` is selected then support for a special
character device at ``/dev/syslog`` is supported.
The function ``syslog_register()`` can be used to register
that character device:

.. code-block:: c

  /****************************************************************************
   * Name: syslog_register
   *
   * Description:
   *   Register a simple character driver at /dev/syslog whose write() method
   *   will transfer data to the SYSLOG device.  This can be useful if, for
   *   example, you want to redirect the output of a program to the SYSLOG.
   *
   *   NOTE that unlike other syslog output, this data is unformatted raw
   *   byte output with no time-stamping or any other SYSLOG features
   *   supported.
   *
   ****************************************************************************/
  
  void syslog_register(void);


.. note:: Careful... there is overloaded naming here!

A character device can serve as the SYSLOG output channel as described above.
There we were referring to a data path like::

  syslog() -> SYSLOG channel layer -> Character driver output channel.

Here we are talking about a different SYSLOG character devices that provides
input data to the SYSLOG channel.
That is a data path like::

  SYSLOG Character driver -> SYSLOG channel layer -> Output channel.

Very confusing and begs for some re-naming.


Using SYSLOG for Debug
======================

Lately, I have starting thinking a little more about the ``SYSLOG`` functions
in general (as well as all of those debug macros). I think I am coming to the
conclusion that there needs to be some things done.

The original design of ``syslog()`` was just some hooks for simple serial
debug output. These simple debug hooks were standardized and renamed
``syslog()`` so that they provide a portable debug interface.
As NuttX has increased in complexity and sophistication over the years,
the implementation of ``syslog()`` "under the hood" is still that mindlessly
simple serial debug logic from the original NuttX release.
Perhaps the time as come to assess what is wrong with the solution
and to implement some improvements in the ``syslog()`` design?

Here are some of the issues of the current implementation that bother me:

Use of File Descriptors
-----------------------

In the default case, the debug output goes out on file descriptor ``1``
(``stdout``). But if you think about that, it is a little crazy.

Each task can have I/O redirected in various ways. In most simple systems,
a serial console is used for stdout in all tasks and the debug output goes
to the serial console so everything is seamless. But if you are using
a mixture of serial consoles, USB serial consoles, telnet sessions, etc.
then who know were where the output is going to go in the most general case.
Bits and pieces could go to different devices.

If you are redirecting stdout to a file, for example, then the debug
information could go into your file, corrupting the output that you wanted.
As another perverse example, try enabling network debug output using a Telnet
session. That is an interesting exercise for anyone who like to see
infinite loops: Telnet I/O generates debug output, the debug output goes
to the Telnet network connection, which generates more network debug output,
and on and one.

Interrupt Handlers
------------------

Output from interrupt handlers, of course, cannot use the console device
at all. File descriptor I/O is not permitted from interrupt handlers.
Attempts to do "normal" SYSLOG output from interrupt handlers will
just result in the output going to the bit bucket.

Low-Level Serial Driver
-----------------------

The usual workaround to get debug output from interrupt handlers is to use
the low-level serial I/O from interrupt handlers.
But there are issues with this as well:

* First, the console may not be the same serial device.
  It might be something else althogether. That means that non-interrupt
  SYSLOG output goes output one way and interrupt level SYSLOG output
  always goes out the serial port.
  Potentially very strange behavior could result.
* Second, the low-level serial output does a busy wait poll!
  That interferes badly with the behavior of the interrupt handler –
  it has to wait within the interrupt handler while the serial output
  is performed. That wait would will be many milliseconds with
  interrupts disabled!
* The low level serial output also interacts the serial driver itself
  making other use of the console impossible. Even, in some cases
  on some platforms, locking up the serial driver.

Interrupt Buffer
----------------

Another way to handle interrupt level output is supported.
This is the only other option available if a Serial Console is not being used.
This second option is enabled with ``CONFIG_SYSLOG_INTBUFFER``.

In this case, syslog output generated from interrupt level logic will simply
be buffered in memory. Then, later, when the next non-interrupt level syslog
output is generated, the buffer interrupt level output will performed.
This works because it essentially defers the syslog output generated
from interrupt handlers until the next opportunity to perform normal output.

Asynchronous Output
-------------------

Another syslog related issue is the asynchronous behavior with debug output
from interrupt handlers. The normal debug output goes to the serial driver
and is buffered for sending there. The size of the serial RX buffer
is configurable. At any given point in time, the current output is behind
realtme depending on that buffer size and the serial BAUD.

The interrupt debug output does not use the serial driver but immdiately
commandeers the serial port and outputs dara in real time.
The result is it appears to happen earlier in the output.

This is also why you lose the last debug output on a crash...
the last debug data is stranded in the serial drivers RX buffer.

Interleaved Output
------------------

Because the output is done character at a time, the debug output
from different tasks may get multiplexed and unreadable in the most critical
of cases. The interleaved output can become totally useless, usually
in the most complex situations where you need the debug output the most.
Many times my plans to debug a problem with SYSLOG output has been thwarted
because the output is just uninterpretable in a highly multitasking context.

Buffer Overrun
--------------

The root cause of this problem is the RX buffering in the serial driver:
Character output is done one character at a time. That is not usually
an issue. But when the system is very busy and he serial RX buffer becomes
full, then each character output causes the caller to suspending,
waiting for space in the RX buffer. It suspends and is moved to the last
of the FIFO for that priority.

When this happens, many tasks may be suspended waiting space to put the next
byte in the serial RX buffer. If the tasks are the same priority, then
the output will be interleaved as described since each task gets essentially
round-robin access to the serial driver. If one of the tasks is lower
priority, then its its output may be deferred for some time until all
of the higher priority tasks complete their output.
Again, leaving a big time skew in the output data.

When there is is a big time skew in the debug output – whether from
asynchronous output from interrupt handlers or from blocked,
lower priority tasks – This can lead to misinterpretation of the
debug output since our instinct is to treat the output as if it were
in sequential order in time.

.. note::

  Although a simple working around for a partical debug scenario
  is simply to increase the size of the RX buffer in the serial
  driver. Assuming that the RX buffer overrun is only the result
  of short-term, bursty behavior, then the large buffer migh prevent
  tasks blocking waiting for space to write the next byte.

Of course, there is no work around for the perverse case where the debug
output is generated at a higher rate than can be transferred
on the serial port. You are just basically out-of-luck in that case.

Solutions
---------

Serialization Buffer
^^^^^^^^^^^^^^^^^^^^

Some of these asynchrony problems could also be reduced or eliminated
if all debug output were buffered in an in-memory FIFO. That buffer
would serialize output from diffrent sources:
Various tasks and interrupt level logic.

There is already a outgoing, RX buffer in the serial driver.
Why would an additional layer of buffering help? Only because then
the interrupt handler debug output could also be serialized.
The input to the FIFO is debug output from all concurrent tasks and interrupt
handlers; the single output of the FIFO would be the serial driver.

Serialization via syslog buffer has been recently implemented in NuttX,
this option is enabled with ``CONFIG_SYSLOG_BUFFER``.

Crash Dump
^^^^^^^^^^

It might even be possible to flush that serialization buffer at the time
of the crash. This has ability has not yet been implemented
as of the time of this writing.

CONFIG_SYSLOG and the RAMLOG
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

There is a partial solution for all of these issues when ``CONFIG_SYSLOG=y``.
In that case, stdout is not used but instead some custom logic is used
that is established by the configuration. Currently that option is only used
with the ``RAMLOG``. The RAMLOG actually works very nicely and eliminates
most of the above issue (except for the interleaving issue).
But the ``RAMLOG`` also requires some additional logic to get the debug
information out of the ``RAMLOG``. In ``NSH``, you can use the dmesg command
to dump the content of the ``RAMLOG`` to the ``NSH`` console.

But I am thinking that some in-memory buffering and serialization
is the solution to most of the issues mentioned about:
That would end the reliance on stdout. It would be compatible with
debug output from interrupt handlers; interrupt handler output
would be serialized in the in-memory buffer in the correct position.

The same interleaving problem could still occur. Adding more buffering
can always eliminate the interleaving problem, provided that the problem
is due to bursty, high-volume output. But you can just increase the size
of the serial RX buffer to accomplish that; any special serialization
is of no help (other than it effectively increases the size of the RX buffer).

The existing ``RAMLOG`` code could be extended so that the buffered data
is agressively dumped to the ``SYSLOG`` device and I think all problems
would be eliminated. The buffer could be dumped that the end of each
non-interrupt level execution of ``syslog()`` (and also on a crash).
Interrupt level output would have to pend in the buffer until the next,
non-interrupt level debug output pushes it out to the serial driver.
