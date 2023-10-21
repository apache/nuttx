``watcher`` Watcher & Watched
=============================

The watcher and watched examples are designed to work together. The watched
example will only appear after watcher is selected.
The watcher is a task that will monitor other tasks that subscribe to be watched.
If a watched task doesn't signal the watcher during the watchdog time period,
the watchdog timer will expire and the watcher will print the tasks that did
not signal and the ones that signaled. The tasks that did not signal will be printed
as the tasks that starved the dog and the tasks that signaled will be printed as
the tasks that fed the dog.
The watcher task will only feed the watchdog timer when all subscribed tasks have
asked to feed dog.

To start the watcher, just run:

``watcher``

The watched example is not required to use the watcher. The watched example is simply
a task that creates 4 tasks that will subscribe to be watched. The first and fourth
will not feed the dog to expose the functionality. This example will show the user
how to subscribe, to feed the dog and to unsubscribe.

To start the watched, just run:

``watched``

P.S: This example will only be supported by the chips that support interrupt on
timeout, i.e., which have the \"capture\" command implemented.

This test depends on these specific configurations settings (your
specific watchdog hardware settings might require additional settings).

- ``CONFIG_EXAMPLES_WATCHER`` – Includes this example.
- ``CONFIG_WATCHDOG`` – Enables watchdog timer support.
- ``CONFIG_NSH_BUILTIN_APPS`` – Build this example an NSH built-in
  function.
- ``CONFIG_DRIVERS_NOTE`` and ``CONFIG_SCHED_INSTRUMENTATION`` – Allows the watcher
  to get the tasks' names.
- ``CONFIG_FS_FAT`` – Allows the creation of a FAT filesystem on the ramdisk
  to create a file with all the necessary info for the watched tasks.

Specific configuration options for the ``watcher`` example include:

- ``CONFIG_EXAMPLES_WATCHER_PRIORITY`` – Watcher Task Priority.
- ``CONFIG_EXAMPLES_WATCHER_STACKSIZE`` – Watcher Task Stack Size.
- ``CONFIG_EXAMPLES_WATCHER_DEVPATH`` – The path to the Watchdog device used by
  the Watcher. Default: ``/dev/watchdog0``.
- ``CONFIG_EXAMPLES_WATCHER_TIMEOUT`` – The watchdog timeout value in
  milliseconds.
- ``CONFIG_EXAMPLES_WATCHER_SIGNAL`` – This is the Signal Number used for
  communication between the watcher task and the watched tasks.

Specific configuration options for the ``watched`` example include:

- ``CONFIG_EXAMPLES_WATCHED_PRIORITY`` – Watched Task Priority.
- ``CONFIG_EXAMPLES_WATCHED_STACKSIZE`` – Watched Task Stack Size.
