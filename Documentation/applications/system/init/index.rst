======================
``init`` Init
======================

Overview
========

In NuttX, there are various options for the init entry point, with NSH being
one of the commonly used ones. However, taking NSH as an example, it is not
suitable for all usage scenarios. For instance, the code size required by
NSH and NSH_LIBRARY is relatively large, and NSH lacks management
capabilities for daemons/services (such as restarting services).

Therefore, we have newly implemented an Init component that is compatible
with most of the syntax of Android Init. It is lightweight, supports
command execution triggered by events, supports service/daemon management,
and its configuration files support include/import functionalities.

Init.rc consists of five categories of statements, all line-oriented with
parameters separated by spaces. The content is processed by a preprocessor,
following C language specifications for escape rules and comment formats.
File path: /etc/init.d/init.rc.

1. Actions and Commands
-----------------------
.. code-block::

    on <trigger>
       <command>
       <command>
       <command>

- An Action (command sequence) contains trigger conditions. When satisfied,
  it is added to the execution queue.
- Commands are the specific execution content of actions, such as setprop
  (set system properties), start (start services), and mount (mount file
  systems).

2. Services and Options
-----------------------
.. code-block::

    service <name> <pathname> [ <argument> ]*
       <option>
       <option>
       ...

- A Service is a program started by Init, with a unique name (can be overridden
  via the override option).
- Options are modifiers for services, affecting their running mode and timing.
  Examples include class (specify service category for batch start/stop),
  override (override previously defined services), restart_period (interval
  for restarting exited services), and reboot_on_failure (critical services
  trigger device reboot on startup failure or abnormal exit).

3. Imports
----------
Used to import other configuration files, supporting single files or all files
in a directory (non-recursive).

.. code-block::

    import <path>

In addition, since init.rc undergoes compiler preprocessing, we can also
include other configuration files during the preprocessing stage using
the #include directive.

.. code-block::

    #include "example.rc"

Triggers
========

The triggers of an action serve as its triggering conditions, supporting
one or more triggers (which must all be satisfied simultaneously).

There are two types of triggers: event triggers (e.g., boot) and action
triggers (also known as property triggers, e.g., property:a=b). Actions can
include multiple action triggers but only one event trigger. Triggers can be
combined with && to represent "AND" conditions.

Note: Currently, only event triggers are supported; action triggers will be
uploaded as soon as possible.

1. Event Trigger Execution Order
--------------------------------
1. boot: The first event after Init starts.
2. init: After BOARDIOC_INIT completes.
3. netinit: Optional, after netinit_bringup() returns.
4. finalinit: After BOARDIOC_FINALINIT completes.

2. Action Triggers Checking
---------------------------
- All action triggers are automatically checked once at startup.
- Property triggers are checked when the property is created or its value
  is updated (e.g., property:a=b is checked when a's value changes).

Commands
========
The commands supported by an action fall into three types: the built-in
commands of Init, the built-in commands of NSH (if NSH is enabled), and
Builtin Apps.

The following is an explanation of some of Init's built-in commands
(some are still under development).

- System Operations: mount/umount (mount/unmount file systems), setprop
  (set system properties), chmod/chown (modify file permissions/owner).
- Service Management: start/stop/restart (start/stop/restart services),
  class_start/class_stop (batch start/stop service categories).
- File Operations: mkdir (create directories with configurable permissions,
  owner, and encryption settings), copy/write (copy files/write file content).
- Others: trigger (trigger events), wait (wait for file existence or timeout),
  loglevel (set Init log level).

Examples
========
This is an example of enabling the basic functions of the Init component,
with all log levels additionally enabled to facilitate debugging of the
init.rc script.

defconfig:

.. code-block:: diff

    +CONFIG_INIT_ENTRYPOINT="init_main"
    +CONFIG_SYSTEM_INIT=y
    +CONFIG_SYSTEM_INIT_DEBUG=y
    +CONFIG_SYSTEM_INIT_INFO=y
    +CONFIG_SYSTEM_INIT_WARN=y

init.rc:

.. code-block::

    on boot
        start console        /* Start the service named "console" */
        sleep 1              /* Block for 1 second */
        exec_start mkdir_tmp /* Start the "mkdir_tmp" service and wait for exit */
        start fastboot

    service console sh
        class core
        restart_period 1000  /* Service restart interval (in milliseconds), calculated from service start time */

    service fastboot fastbootd
        class core
        restart_period 7000

    service mkdir_tmp sh -c "mkdir /tmp"
        reboot_on_failure 0 /* Reboot device with reason 0 on startup failure or abnormal exit */
        oneshot             /* This service runs only once; releases resources after exit */

    import /etc/init.d/test.rc /* Include test.rc at runtime */
    #include "test.rc"         /* Include test.rc during preprocessing */
