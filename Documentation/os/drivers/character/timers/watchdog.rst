======================
Watchdog Timer Drivers
======================

NuttX supports a low-level, two-part watchdog timer driver.

#. An "upper half", generic driver that provides the common
   watchdog timer interface to application level code, and
#. A "lower half", platform-specific driver that implements the
   low-level timer controls to implement the watchdog timer
   functionality.

Files supporting the watchdog timer driver can be found in the
following locations:

-  **Interface Definition**. The header file for the NuttX
   watchdog timer driver reside at
   ``include/nuttx/timers/watchdog.h``. This header file includes
   both the application level interface to the watchdog timer
   driver as well as the interface between the "upper half" and
   "lower half" drivers. The watchdog timer driver uses a standard
   character driver framework.
-  **"Upper Half" Driver**. The generic, "upper half" watchdog
   timer driver resides at ``drivers/timers/watchdog.c``.
-  **"Lower Half" Drivers**. Platform-specific watchdog timer
   drivers reside in
   ``arch/<architecture>//src/<hardware>`` directory
   for the specific processor ``<architecture>`` and for the
   specific ``<chip>`` watchdog timer peripheral devices.

There are two ways to enable Watchdog Timer Support along with the Watchdog Example.
The first is faster and simpler. Just run the following command to use a ready config
file with watchdog timer support and example included. You need to check if there's a
watchdog config file for your specific chip. You may check it at the specific board's
path: ``/boards/<arch>/<chip>/<board>/config``.

.. code-block:: console

   $ ./tools/configure.sh <board>:watchdog

And the second way is creating your own config file. To do so, follow the next
instructions.

Enabling the Watchdog Support and Example in ``menuconfing``
------------------------------------------------------------

1. Select Watchdog Timer Instances

   To select these wdts browse in the ``menuconfig`` using the following path:

   Go into menu :menuselection:`System Type --> <Chip> Peripheral Selection` and
   press :kbd:`Enter`. Then select one or more watchdog timers according to
   availability of your chip.

2. Enable the Watchdog Timer Support

   Go into menu :menuselection:`Device Drivers --> Timer Driver Support` and press
   :kbd:`Enter`. Then enable:

   - [x] Watchdog Timer Support

3. Include the Watchdog Timer Example

   Go into menu :menuselection:`Application Configuration --> Examples` and press
   :kbd:`Enter`. Then select the Watchdog Timer example.

 - [x] Watchdog Timer example

Below the option, it is possible to manually configure some standard parameters
that will be used by the example, but they also can be passed as command line
arguments later.
The parameters are the following: the standard timer device path (which defines
the WDT instance), the timeout period (which is the period on which the watchdog
will expire), the ping delay (which is the interval period between feeding the dog)
and the ping time (which is the total interval that the example will feed the dog,
after this interval, the dog will starve and the chip will trigger an interrupt or reset.

4. Include the Debug Watchdog Feature

   In order to get the watchdog timer status, you need to enable it. For production
   code and for your application you may disable it.

   Go into menu :menuselection:`Build Setup --> Debug Options` and press :kbd:`Enter`. Then enable:

   - [x] Enable Debug Features
   - [x] Watchdog Timer Debug Features

Watchdog Timer Example
----------------------

The previously selected example will basically do the following:

* Open the watchdog device
* Set the watchdog timeout
* Start the watchdog timer
* Ping (feed the dog) during the ``pingtime`` with a delay of ``pingdelay`` and
  print out the wdt status in case debug was enabled.
* Enter into an endless loop without pinging. It will cause the watchdog timer
  to reset the chip on timeout, i.e., after timer expiration.

The `example code <https://github.com/apache/nuttx-apps/blob/master/examples/watchdog/watchdog_main.c>`_
may be explored, its path is at ``/examples/watchdog/watchdog_main.c`` in the
apps' repository.

In NuttX, the watchdog timer driver is a character driver and when a chip supports
multiple watchdog timers, each one is accessible through its respective special file
in ``/dev`` directory. Each watchdog timer is registered using a unique numeric
identifier (i.e. ``/dev/watchdog0``, ``/dev/watchdog1``, ...).

Use the following command to run the example:

.. code-block:: console

  nsh> wdog

This command will use the watchdog timer 0. To use the others, specify it through
a parameter (where x is the timer number):

.. code-block:: console

  nsh> wdog -i /dev/watchdogx

Additionally, the watchdog driver can be read by using standard commands such as ``cat``.
It will emit relevant information about the watchdog and its current status, including the
flags bitset, timeout and timeleft from the current slice as milliseconds:

.. code-block:: console

  nsh> cat /dev/watchdogx
  Status     :
  flags      : 0x00000000
  timeout    : 30000
  timeleft   : 9889

Application Level Interface
----------------------------

The first necessary thing to be done in order to use the watchdog timer driver
in an application is to include the header file for the NuttX Watchdog timer
driver. It contains the Application Level Interface to the timer driver. To do so,
include:

.. code-block:: c

  #include <nuttx/timers/watchdog.h>

At an application level, the watchdog timer functionalities may be accessed through
``ioctl`` systems calls. These ``ioctl`` commands internally call lower-half layer
operations and the parameters are forwarded to these operations through the ``ioctl``
system call. The example provides a great resource to demonstrate how to use those
``ioctl`` commands. The available ``ioctl`` commands are:

.. c:macro:: WDIOC_START

This command starts the watchdog timer.

.. c:macro:: WDIOC_STOP

This command stops the watchdog timer.

.. c:macro:: WDIOC_GETSTATUS

This command gets the status of the watchdog timer. It receives a writeable
pointer to struct ``watchdog_status_s`` as parameter. The lower-half driver
writes the current status in this struct.

.. c:struct:: watchdog_status_s
.. code-block:: c

	struct watchdog_status_s
	{
	  uint32_t  flags;          /* See WDFLAGS_* definitions above */
	  uint32_t  timeout;        /* The current timeout setting (in milliseconds) */
	  uint32_t  timeleft;       /* Time left until the watchdog expiration
		                     * (in milliseconds) */
	};

.. c:macro:: WDIOC_SETTIMEOUT

This command sets the timeout value, i.e., the value that will trigger the reset
or interrupt. The argument is a ``uint32_t`` value in milliseconds.

.. c:macro:: WDIOC_CAPTURE

This command registers an user callback that will be triggered on timeout. It
receives as argument a pointer to struct ``watchdog_capture_s``. If the user
callback is NULL, then it configures only to reset. Not all chips support
interrupt on timeout. This command is optional, i.e., if it's not used, the
standard behaviour is to reset on timeout.

.. c:struct:: watchdog_capture_s
.. code-block:: c

	struct watchdog_capture_s
	{
	  CODE xcpt_t newhandler;   /* The new watchdog capture handler */
	  CODE xcpt_t oldhandler;   /* The previous watchdog capture handler (if any) */
	};

.. c:macro:: WDIOC_KEEPALIVE

 This command resets the watchdog timer AKA '**ping**", "**kick**", "**pet**",  "**feed**" the dog".

Enable Built in System Monitoring to reset the watchdog
-------------------------------------------------------

The auto-monitor provides an OS-internal mechanism to automatically start and
repeatedly reset the watchdog.

To enable it, follow the next instructions:

1. Select a Watchdog Timer Instance

 To select the wdt browse in the ``menuconfig`` using the following path:

 Go into menu :menuselection:`System Type --> <Chip> Peripheral Selection` and
 press :kbd:`Enter`. Then select one watchdog timer.

2. Enable the Auto-monitor option

   Go into menu :menuselection:`Device Drivers --> Timer Driver Support` and press
   :kbd:`Enter`. Then enable:

   - [x] Watchdog Timer Support

   Then press :kbd:`Enter` again to enter into the Watchdog Timer Support menu. And
   finally enable the Auto-monitor option:

   - [x] Auto-monitor

   After selecting the option you may want to configure some parameters:

   * **Timeout**: It is the watchdog timer expiration time in seconds.
   * **Keep a live interval**: This is the interval in which the watchdog will be
     fed. It is in seconds. It can't be bigger than the timeout. If this interval
     is equal to timeout interval, than this interval will automatically change to
     half timeout.
   * **Keep alive by**: This is a choice to determine who is going to feed the dog.
     There are 4 possible choices that are described as follows.

     ``Capture callback``: This choice registers a watchdog timer callback to reset
     the watchdog every time it expires, i.e., on timeout.

     ``Timer callback``: This choice also uses a timer callback to reset the watchdog,
     but it will reset the watchdog every "keep a live interval".

     ``Worker callback``:  This choice uses a Work Queue to reset the watchdog every
     "keep a live interval". This choice depends on having the Low or High Priority
     Work Queue enabled.
     If only the High Priority Work Queue is enabled, this one will be used, otherwise
     Low Priority Work Queue is used.

     So, before enabling it, go into menu :menuselection:`RTOS Features --> Work queue support`
     and press :kbd:`Enter`.

     - [x] Low priority (kernel) worker thread

     ``Idle callback``: This choice sets an Idle callback to feed the dog. It depends
     on the PM module, because this callback is triggered by the PM state change.
     To enable it do the following:

     Go into menu :menuselection:`Device Drivers` and enable:

     - [x] Power Management Support

     After selecting one of these choices, the chip will keep itself alive by one of
     these options.

Software Watchdog Implementation (watchdog_wdog)
------------------------------------------------

The ``CONFIG_WATCHDOG_WDOG`` option provides a watchdog "lower half" built on
the NuttX internal wdog timers, for the platforms that have no watchdog
hardware, such as the simulator. It is the watchdog counterpart of
``timer_wdog``. Like any watchdog it expires if it is not fed within its
timeout, but since there is no hardware to reset the system, it does what
``CONFIG_WATCHDOG_WDOG_ACTION`` selects: panic, reboot (``board_reset()``) or
power off (``board_power_off()``). With the last two, ``CONFIG_WATCHDOG_WDOG_STATUS``
is the status given to the board.

Because it relies on the system tick, it can not detect a system where the tick
stopped. It is not the same as ``CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG``, where a
wdog feeds a real watchdog, and it is not available with the oneshot and timer
auto-monitors.

Call ``watchdog_wdog_initialize()`` from the board initialization code to
register it. The initial timeout is ``CONFIG_WATCHDOG_WDOG_TIMEOUT``:

.. code-block:: c

   #include <nuttx/timers/watchdog_wdog.h>

   #ifdef CONFIG_WATCHDOG_WDOG
     ret = watchdog_wdog_initialize(CONFIG_WATCHDOG_DEVPATH);
     if (ret < 0)
       {
         syslog(LOG_ERR, "ERROR: watchdog_wdog_initialize failed: %d\n", ret);
       }
   #endif

The simulator uses it, see the ``watchdog`` configuration of the
:doc:`sim </platforms/sim/sim/boards/sim/index>`.

Task Supervision
----------------

``CONFIG_WATCHDOG_AUTOMONITOR_TASKS`` makes the auto-monitor supervise the
tasks that use a watchdog device. The auto-monitor feeds the hardware watchdog,
with the keepalive method selected for it, only while every task feeds the
device in time, so a task that hangs, exits or crashes makes the system reset.
Without it, the auto-monitor feeds the hardware watchdog unconditionally.

Each ``open()`` creates a *channel*, armed with
``CONFIG_WATCHDOG_AUTOMONITOR_TASKS_TIMEOUT``. The commands follow the Linux
watchdog API but act on the channel, never on the hardware watchdog:

=========================== ==================================================
Operation                   Effect on the channel
=========================== ==================================================
``write()``                 Feeds the channel. A buffer with a ``'V'``
                            disarms it instead, if ``CONFIG_WATCHDOG_MAGIC_V``
                            is set.
``WDIOC_KEEPALIVE``         Feeds the channel.
``WDIOC_SETTIMEOUT``        Sets the time-out in milliseconds and feeds the
                            channel. Zero fails with ``EINVAL``.
``WDIOC_GETSTATUS``         Returns the flags, the time-out and the time left.
``WDIOC_START``             Arms the channel again.
``WDIOC_STOP``              Disarms the channel, same as writing a ``'V'``.
``WDIOC_CAPTURE``, others   Fail with ``ENOSYS`` and ``ENOTTY``.
``close()``                 Releases the channel only if it is disarmed.
=========================== ==================================================

A channel closed while armed is never fed again, so a task that exits or is
killed without writing a ``'V'`` makes the channel time out, as on Linux.

.. code-block:: c

   int fd = open(CONFIG_WATCHDOG_DEVPATH, O_WRONLY);

   ioctl(fd, WDIOC_SETTIMEOUT, 2000);   /* 2 s */

   while (running)
     {
       do_work();
       write(fd, "x", 1);               /* feed */
     }

   write(fd, "V", 1);                   /* done, stop supervising */
   close(fd);

The hardware watchdog is armed when the device is registered and fed even
with no channel, so the system also resets if the keepalive method stops, for
example the low priority work queue with ``CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER``.
When a channel times out the auto-monitor logs the pid and the name of its task
and stops feeding, or panics with ``CONFIG_WATCHDOG_AUTOMONITOR_TASKS_PANIC``.
A time-out is detected up to one keepalive interval late
(``CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL``), but a channel that timed out
can not be recovered by feeding, disarming or arming it. The time-out of a
channel is independent of the hardware time-out
(``CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT``), which only limits how long the
keepalive method can stay stopped. The keepalive methods that run in interrupt
context are supported, the channels are protected by a spinlock. The tasks to supervise have to open the device early, because the
auto-monitor only knows the tasks that did.

The keepalive method also decides what else makes the system reset, besides a
channel that times out:

=============== ===================================================================
Method          Also resets the system when
=============== ===================================================================
``BY_WORKER``   The low priority work queue stops, for example a task that locks
                the scheduler or a priority inversion that starves it.
``BY_WDOG``     The system tick stops. It runs in interrupt context, so a task that
                locks the scheduler is not detected.
``BY_TIMER``,   The timer or oneshot interrupt stops. They run in interrupt
``BY_ONESHOT``  context like ``BY_WDOG``.
``BY_CAPTURE``  The hardware watchdog interrupt is not served. The channels are
                checked only when the early warning interrupt happens, which is
                the latest possible moment.
``BY_IDLE``     The system never goes idle, so it must not be busy all the time
                with this method.
=============== ===================================================================

The ``taskwdt_test`` application (``CONFIG_TESTING_TASKWDT``) exercises the
supervision: run it without arguments for the list of test cases.
