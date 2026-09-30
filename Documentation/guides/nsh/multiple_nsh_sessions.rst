=====================
Multiple NSH Sessions
=====================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Multiple+NSH+Sessions

Q:  
  I would like to run the NuttShell on multiple serial ports, but haven't
  figured it out yet; can you point me in the right direction?

A:  
  Easy. Don't use ``apps/examples/nsh_main.c``. Create your own main function
  something like this (with all error handling omitted for simplicity). By the
  way, this is all standard POSIX stuff that you can get detailed information
  about by just Googling `dup2` or maybe `I/O redirection`:

.. code-block:: c

   int my_main(int argc, char **argv)
   {
     const char *tty = argv[1];
     int fd = open(tty, O_RDWR);
     (void)dup2(fd, 0);
     (void)dup2(fd, 1);
     (void)dup2(fd, 2);
     close(fd);
     ...
   }

And the rest is just like the original ``nsh_main()`` function (in fact,
perhaps the existing ``nsh_main()`` function could be optionally extended to
accept a console device string?). Then you can start a new NSH session on any
TTY like:

.. code-block:: none

   nsh> mynsh /dev/ttyS2 &

This should cause a new NSH session to appear on ``ttyS2``. That session will
persist until you do the following from the new session:

.. code-block:: none

   nsh> exit

Then the new session, i.e., ``my_main()`` will exit.

If you were to do something like:

.. code-block:: none

   nsh> mynsh /dev/console

then you would get nested NSH sessions on the same console. The first session
would halt and wait for the second session to take control of the console until
it exits. Then the first session will take over console again.

NuTTY
=====

In a previous discussion, there was talk about implementing the moral equivalent
of getty in NuttX (of course, it would be called "nutty"). A simple
implementation of nutty would work like this:

1. It would wait on ``poll()`` on every (configured) serial device.
2. Whenever it is awakened, it would start something like ``my_main()`` above
   on the active serial port.
3. NSH has an option to enable logins, but it would be better to remove the
   existing login information from NSH and centralize it in nutty.

That way, you could connect to any TTY, hit enter, and you would get an NSH
session. Hmm... it is not clear how nutty would get the TTY back after the
session is closed. That part may require some additional thought.

Other Ideas
===========

There are other ways to get multiple NSH sessions:

- Telnet already supports multiple sessions.
- Implement the existing NSH as an ELF program, then you can get multiple NSH
  sessions with ``posix_spawn`` by simply redirecting I/O.
- Using the tiny NxWM window managers, multiple NSH windows are already
  supported.
