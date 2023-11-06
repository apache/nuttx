=========================
CNxConsole Keyboard Input
=========================

**Players**

Let's look at the major players in the keyboard data transfer. This is much more
complex than you might initially think:

**Special Drivers**
NxConsole Device. The NX console input comes through a special device driver that
is registered at ``/dev/nxcon0`` as part of early initialization.

**Kernel Threads**

* **NX Server Thread** The NX Server is the graphics server command. It receives
  messages from various sources, performs graphics actions, and forwards graphic
  event messages to the correct window. Most of the time, the NX Server Thread was
  waiting on a message queue to receive the next graphics event.

* **NxConsole Threads** Each NxConsole has a thread that was started when each
  ``NxWM::CNxConsole`` instance was created by NxWM. Each ``NxWM::CNxConsole``
  thread opens the NxConsole driver at ``/dev/nxcon0`` and redirects stdin,
  stdout, and stderr to/from that special device. Normally, the ``NxWM::CNxConsole``
  thread is stopped, just waiting on read for keyboard input to complete.

**Application Threads**

* **NxWidgets Window Event Handler Thread** ``CNxServer::listener()`` is an
  application thread started by NxWidgets each time a new window is opened.
  It receives window messages from the NX server and dispatches the messages
  accordingly.

* **Keyboard Listener Thread** ``CKeyboard::listener()`` is an application thread
  that is started by NxWM. It just listens for keyboard input and forwards it through
  the graphics routing system.

Now here is the sequence of events to get keyboard input from the stdin device to
the correct NxConsole.

#. Application Space / NxWidgets Window Event Handler Thread
   Let's start with the initial state of the NX Server Thread. Initially, it will
   just want for messages from the NX Server.

     * ``NxWidgets/libnxwidgets/src/cnxserver.cxx``
       ``CNxServer::listener()`` is it window listener thread. It just calls
       ``nx_eventhandler()`` to receive and process server events. There is one
       such listener thread per window.

     * ``nuttx/libnx/nxwm/nx_eventhandler``
       ``nx_eventhandler()`` waits to receive a message from the NX server. Each
       window has its own message queue; each window instance has its own
       ``nx_eventhandler()`` waiting for messages.

#. Application Space / Keyboard Listener Thread

   Here are the immediate events that happen when the keyboard data is entered.
   The Keyboard Listener Thread wakes up and forwards the Keyboard data to the
   the NX Server. Only the NX Server knows which window should get the keyboard input.

     * ``NxWidgets\nxwm\src\ckeyboard.cxx``
       ``CKeyboard::listener()`` is a tiny thread that is started by NxWM that just
       listens for keyboard input. It opens the keyboard device and waits on a ``read()``
       from the keyboard device to receive the next keyboard input. When data is
       returned by reading from the keyboard device, ``CKeyboard::listener()``
       calls ``nx_kbdin()``

     * ``nuttx\libnx\nxmu\nx_kbdin.c``
       This library function just hides the NX server messaging implementation.
       It sends the ``NX_SVRMSG_KBDIN`` to the NX server thread.

#. Kernel Space / NX Server

   The NX Server wakes up, receives the keyboard message, and forwards it to the
   appropriate window.

     * ``nuttx/graphics/nxmu/nxmu/nxmu_server.c``
       The receipt of the ``NX_SVRMSG_KBDIN`` message wakes up the NX server
       thread. The NX server thread decodes the message and calls ``nxmu_kbdin()``.

     * ``nuttx/graphics/nxconsole/nxmu_kbdin.c``
       ``nxmu_kbdin()`` simply sends the ``NX_CLIMSG_KBDIN`` to the appropriate
       windows client via the client message queue associated with the window.

#. Application Space / NxWidgets Window Event Handler Thread

   The Windows client wakes up when the keyboard message is received. It forwards
   the keyboard data to ``/dev/nxcon0/`` so that is can be available to the
   NxConsole window.

     * ``nuttx/libnx/nxwm/nx_eventhandler``
       The ``nx_eventhandler()`` logic running in the ``CNxServer::listener()``
       thread receives the ``NX_CLIMSG_KBDIN`` message and dispatches it to the
       kbdin callback method. In this case that callback method maps to
       ``CCallback::newKeyboardEvent()``.

     * ``NxWidgets/libnxwidgets/src/ccallback.cxx``
       For normal keyboard input, ``CCallback::newKeyboardEvent()`` directs the
       Keyboard to the widget with focus via the ``CWidgetControl::newKeyboardEvent()``
       method. But the story is different for the NxConsole window. This case,
       ``CCallback::newKeyboardEvent()``, calls ``nxcon_kbdin()``.

     * ``nuttx/graphics/nxconsole/nxcon_kbdin.c``
       ``nxcon_kbdin()`` adds the keyboard data to a circular buffer and wakes up
       any reads on the ``/dev/nxcon0`` input device.


   NOTE: Here is a violation of the Application and Kernel Space boundaries.
   ``nxcon_kbdin.c`` built into Kernel Space but it is called from Application
   Space. The solution is to (1) move ``nxcon_kbdin()`` to ``libnx/`` and (2) it
   should then communicate with the ``/dev/nxcon9`` driver via ioctl calls.
   This will become a problem some day.

#. Kernel Space / NxConsole Thread

   Finally,
     * ``nuttx/graphics/nxconsole/nxcon_kbdin.c``
       The receipt and enqueuing of keyboard data by ``nxcon_kbdin()`` wakes up
       any threads waiting in the ``nxcon_read()`` method. This is how the
       NxConsole gets its keyboard input.


**Mouse Input**
Almost everything said here applies to mouse/touchscreen input as well. If we
were to replace the names keyboard to mouse, kbdin to mousein, etc. you have a
pretty good description of how mouse/touchscreen input works.

The mouse/touchscreen input is a little simpler, however: The main simplication
is that the additional complexities of the NxConsole and its special input device
do not apply. Mouse/touchscreen inut as always steered to widgets when the
callback is received in ``CCallback::newMouseEvent`` by an unconditional call to
``CWidgetControl::newMouseEvent``. There is a "fork in the road" at the
corresponding point in the logic of ``CCallback::newKeyboardEvent``
