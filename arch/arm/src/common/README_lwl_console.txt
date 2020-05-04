The file arm_lwl_console.c implements a 'Lightweight Link' protocol between
a target and debugger for use when you need a console but the target doesn't
have a spare serial port or other available resource. This implements a
new console type which uses two words of memory for data exchange.

It is not particularly efficient because of the various compromises that are
made (polling in busy loops, mostly) but it works well enough to give you
something where you previously had nothing...typically the case when you're
bring up a new CPU, or when the hardware designer thought the softies could
cope without a logging port. It has an advantage over semi-hosting in that
it doesn't put the target into debug mode while it's running, so you've got
some hope of maintaining real time semantics.  To be clear, for output only
use you'd be better off with SWO if you've got it available!

There is a terminal program in python(*) for the host side in
tools/ocdconsole.py for use with openocd...the NuttX side functionality is
not dependent on a specific debugger, the only requirement on it being that
the debugger can watch and modify a memory location on the target while it is executing.

Typical use is;

$ tools/ocdconsole.py
==Link Activated

NuttShell (NSH)
nsh> help
help usage:  help [-v] [<cmd>]

  ?        echo     exit     hexdump  ls       mh sleep    xd
  cat      exec     help     kill     mb       mw usleep
nsh>

On the target side it's transparent, and is just a console;

nsh> ls /dev
/dev:
 console
 null
 ttyS0
nsh> echo "Hello World" > /dev/console
Hello World
nsh>

CPU load on the host is surprisingly low given that the polling loop is
continuous (probably due to the fact that openocd is spending most of it's
time waiting for messages to/from the debug port on the target). When not
actively doing anything there's no load on the target, but waiting for input
is done in a busy polled loop (so the thread is effectively busy-locked)
and output busy-waits for the previous message to be collected before it
sends the next one.

For now I've only made it available on stm32, but it should only be a case
of changing the Kconfig and Make.defs for other arm CPUs to make it
available for them too. Moving beyond arm needs knowledge of the targets
that I don't have.

If anyone fancies extending this proof-of-concept to full Segger-RTT-style
functionality then drop me a note, there are plenty of ways to improve
performance.
