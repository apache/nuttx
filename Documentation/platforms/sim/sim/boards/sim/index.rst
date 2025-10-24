===
SIM
===

.. tags:: arch:sim

This documentation page describes the contents of the build configurations
available for the NuttX "sim" target. The sim target is a NuttX port that runs
as a user-space program under Linux, Cygwin, or macOS. It is a very "low
fidelity" embedded system simulation: This environment does not support any kind
of asynchronous events; there are nothing like interrupts in this context.
Therefore, there can be no preempting events.

The sim target is used primarily as a development and test platform for new RTOS
features. It is also of academic interest. However, it has no known real-world
application.

Fake Interrupts
---------------

In order to get timed behavior, the system timer "interrupt handler" is called
from the sim target's IDLE loop. The IDLE runs whenever there is no other
task running. So, for example, if a task calls ``sleep()``, then that task will
suspend wanting for the time to elapse. If nothing else is available to run,
then the IDLE loop runs and the timer increments, eventually re-awakening the
sleeping task.

Context switching is based on logic similar to ``setjmp()`` and ``longjmp()``.

Timing Fidelity
---------------

.. note::

   The sim target's IDLE loop to delay on each call so that the system "timer
   interrupt" is called at a rate approximately correct for the system timer
   tick rate. This option can be enabled with ``CONFIG_SIM_WALLTIME_SIGNAL``
   which will drive the entire simulation by using a host timer that ticks at
   ``CONFIG_USEC_PER_TICK``. This option will no longer deliver 'tick' events
   from Idle task and it will generate them from the host signal handler.
   Another option is to use ``CONFIG_SIM_WALLTIME_SLEEP`` which will enable the
   tick events to be delayed from the Idle task by using a host sleep call.

Debugging
=========

One of the best reasons to use the simulation is that is supports great Linux-
based debugging. Here are the steps to follow in order to use the Linux ``ddd``
graphical front-end to GDB:

1. Enable debug symbols by ensuring ``CONFIG_DEBUG_SYMBOLS=y``. You can enable
   this via the Kconfig menu (``make menuconfig``).

2. Re-build NuttX:

   .. code:: console

      $ cd <NuttX-Directory>
      $ make clean
      $ make

3. Then start the debugging:

   .. code:: console

      $ ddd nuttx &
      gdb> b user_start
      gdb> r

.. note::

   This above steps work fine on Linux, Cygwin, and macOS. On Cygwin, you will
   need to start the Cygwin-X server before running ``ddd``. On macOS, it's
   probably easier to use ``lldb`` instead of ``gdb``.

Issues
======

64-Bit Issues
-------------

As mentioned above, context switching is based on logic like ``setjmp()`` and
``longjmp()``. This context switching is available for 32-bit and 64-bit
targets.

You must, however, set the correct target in the configuration before you build:
``CONFIG_HOST_X86_64`` or ``CONFIG_HOST_X86`` for 64- and 32-bit targets,
respectively. On a 64-bit machine, you can also force the 32-bit build with
``CONFIG_SIM_M32=y`` (which does not seem to be supported by more contemporary
x86_64 compilers).

There are other 64-bit issues as well. For example, addresses are retained in
32-bit unsigned integer types in a few places. On a 64-bit machine, the 32- bit
address storage may corrupt 64-bit addressing.

.. note::

    This is really a bug; addresses should not be retained in ``uint32_t`` types
    but rather in ``uintptr_t`` types to avoid issues just like this.

Compiler differences
--------------------

Operator new:

* Problem: 'operator new' takes ``size_t ('...')`` as first parameter
* Workaround: Add ``-fpermissive`` to the compilation flags

Stack Size Issues
-----------------

When you run the NuttX simulation, it uses stacks allocated by NuttX from the
NuttX heap. The memory management model is exactly the same in the simulation
as in a real, target system. This is good because this produces a higher
fidelity simulation.

However, when the simulation calls into the host OS libraries, it will still
use these small simulation stacks. This happens, for example, when you call
into the system to get and put characters to the console window or when you
make X11 calls into the system. The programming model within those libraries
will assume the host OS environment where the stack size grows dynamically
and not the small, limited stacks of a deeply embedded system.

As a consequence, those system libraries may allocate large data structures on
the stack and overflow the small NuttX stacks. X11, in particular, requires
large stacks. If you are using X11 in the simulation, make sure that you set
aside a "lot" of stack for the X11 library calls (maybe 8 or 16Kb). The stack
size for the thread that begins with user start is controlled by the
configuration setting ``CONFIG_INIT_STACKSIZE``; you may need to increase this
value to larger number to survive the X11 library calls.

If you are running X11 applications such as NSH add-on programs, then the stack
size of the add-on program is controlled in another way. Here are the steps for
increasing the stack size in that case:

.. code:: console

   $ cd ../apps/builtin    # Go to the builtin apps directory
   $ vi builtin_list.h     # Edit this file and increase the stack size of the add-on
   $ rm .built *.o         # This will force the builtin apps logic to rebuild

Symbol Collisions
-----------------

The simulation build is a two pass build:

1. On the first pass, an intermediate, partially relocatable object is created
   called ``nuttx.rel``. This includes all of the files that are part of the
   NuttX "domain."

2. On the second pass, the files which are in the host OS domain are built and
   then linked with ``nuttx.rel`` to generate the simulation program.

NuttX is a POSIX compliant RTOS and is normally built on a POSIX compliant host
environment (like Linux, Cygwin, or macOS). As a result, the same symbols are
exported by both the NuttX domain and the host domain. How can we keep them
separate?

This is done using the special file ``nuttx-name.dat``. This file just contains
a mapping of original function names to new function names. For example, the
NuttX ``printf()`` will get the new name ``NXprintf()``.

This ``nuttx-names.dat`` file is used by the ``objcopy`` program between pass 1
and pass 2 to rename all of the symbols in the ``nuttx.rel`` object so that they
do not collide with names provided by the host OS in the host PC domain.

Occasionally, as you test new functionality, you will find that you need to add
more names to the ``nuttx-names.dat`` file. If there is a missing name mapping
in ``nuttx-names.dat``, the symptoms may be very obscure and difficult to debug.
What happens in this case is that when logic in ``nuttx.rel`` intended to call
the NuttX domain function, it instead calls into the host OS function of the
same name.

Often you can survive such events. For example, it really should not matter
which version of ``strlen()`` you call. Other times, it can cause subtle,
mysterious errors. Usually, however, callng the wrong function in the wrong OS
results in a fatal crash.

On macOS, instead of ``objcopy``, ``-unexported_symbols_list`` linker option is
used to hide symbols in the NuttX domain, using the same list of symbols from
``nuttx-name.dat``.

Networking Issues
-----------------

Please issue these commands to setup the reliable network on Ubuntu:

.. code:: console

   $ sudo apt-get -y install net-tools
   $ sudo nuttx/tools/simbridge.sh eth0 on

Here are some tips you may need:

1. Must launch the executable with the root permission
2. Have to use virtual machine if host is in corporation network
3. Configure the network adapter in NAT mode if virtual machine is used

X11 Issues
----------

There is an X11-based framebuffer driver that you can use to exercise the
NuttX graphics subsystem on the simulator (see the sim/nx11 configuration
below). This may require a lot of tinkering to get working, depending upon
where your X11 installation stores libraries and header files and how it names
libraries.

For example, on Ubuntu 9.09, I had to do the following to get a clean build:

.. code:: console

   $ cd /usr/lib/
   $ sudo ln -s libXext.so.6.4.0 libXext.so

.. note::

   I also get a segmentation fault at the conclusion of the NX test; that will
   need to get looked into as well.

.. note::

   You may need issue this command with the latest Ubuntu before launch:

   .. code:: console

      $ sudo xhost +

Cygwin64 Issues
---------------

There are some additional issues using the simulator with Cygwin64. Below is
the summary of the changes that I had to make to get the simulator working in
that environment:

1. ``CONFIG_HOST_X86_64=y``, ``CONFIG_SIM_M32=n``

   Need to select X64_64. Cygwin64 tools do not seem to support any option
   to build a 32-bit target.

2. ``CONFIG_SIM_CYGWIN_DECORATED=n``

   Older versions of Cygwin tools decorated C symbol names by adding an
   underscore to the beginning of the symbol name. Newer versions of Cygwin do
   not seem to do this. Deselecting ```CONFIG_SIM_CYGWIN_DECORATED``` will
   select the symbols without the leading underscore as needed by the Cygwin64
   toolchain.

   How do you know if you need this option? You could look at the generated
   symbol tables to see if there are underscore characters at the beginning
   of the symbol names. Or, if you need this option, the simulation will not
   run. It will crash early, probably in some function due to the failure to
   allocate memory.

   In this case, when I tried to run nutt.exe from the command line, it
   exited silently. Running with GDB I get the following (before hitting a
   breakpoint at ``main()``):

   .. code:: console

      (gdb) r
      Starting program: /cygdrive/c/Users/Gregory/Documents/projects/nuttx/master/nuttx/nuttx.exe
      [New Thread 6512.0xda8]
      [New Thread 6512.0x998]
            1 [main] nuttx 6512 C:\Users\Gregory\Documents\projects\nuttx\master\nuttx\nuttx.exe: *** fatal error - Internal error: Out of memory for new path buf.
          736 [main] nuttx 6512 cygwin_exception::open_stackdumpfile: Dumping stack trace to nuttx.exe.stackdump
      [Thread 6512.0x998 exited with code 256]
      [Inferior 1 (process 6512) exited with code 0400]

4. ``CONFIG_SIM_X8664_SYSTEMV=n``, ``CONFIG_SIM_X8664_MICROSOFT=y``

   Select Microsoft x64 calling convention.

   The Microsoft x64 calling convention is followed on Microsoft Windows and
   pre-boot UEFI (for long mode on x86-64). It uses registers ``RCX``, ``RDX``,
   ``R8``, R9 for the first four integer or pointer arguments (in that order),
   and ``XMM0``, ``XMM1``, ``XMM2``, ``XMM3`` are used for floating point
   arguments. Additional arguments are pushed onto the stack (right to left).
   Integer return values (similar to x86) are returned in RAX if 64 bits or
   less. Floating point return values are returned in ``XMM0``. Parameters less
   than 64 bits long are not zero extended; the high bits are not zeroed.

SMP
---

This configuration has basic support for SMP testing. The simulation supports
the emulation of multiple CPUs by creating multiple pthreads, each running a
copy of the simulation in the same process address space.

You can enable SMP for ostest configuration by enabling these config options:

.. code: diff

   +CONFIG_SPINLOCK=y
   +CONFIG_SMP=y
   +CONFIG_SMP_NCPUS=2

And you can enable some additional debug output with:

.. code:: diff

   -# CONFIG_DEBUG_SCHED is not set
   +CONFIG_DEBUG_SCHED=y

   -# CONFIG_SCHED_INSTRUMENTATION is not set
   -# CONFIG_SCHED_INSTRUMENTATION_SWITCH is not set
   +CONFIG_SCHED_INSTRUMENTATION=y
   +CONFIG_SCHED_INSTRUMENTATION_SWITCH=y

The SMP configuration will run with ``CONFIG_SMP_NCPUS=1``. In this case there
is, of course, no multi-CPU processing, but this does verify the correctness of
some of the basic SMP logic.

BASIC
=====

I have used the ``sim:nsh`` configuration to test Michael Haardt's BASIC
interpreter that you can find at ``apps/interpreters/bas``.

Bas is an interpreter for the classic dialect of the programming language BASIC.
It is pretty compatible to typical BASIC interpreters of the 1980s, unlike some
other UNIX BASIC interpreters, that implement a different syntax, breaking
compatibility to existing programs. Bas offers many ANSI BASIC statements for
structured programming, such as procedures, local variables and various loop
types. Further there are matrix operations, automatic LIST indentation and many
statements and functions found in specific classic dialects. Line numbers are
not required.

There is also a test suite for the interpreter that can be found at
``apps/examples/bastest``.

Usage
-----

This setup will initialize the BASIC test (optional). This will mount a ROMFS
file system at ``/mnt/romfs`` that contains the BASIC test files:

.. code:: console

   nsh> bastest
   Registering romdisk at /dev/ram6
   Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram6
   nsh>

The interactive interpreter is started like:

.. code:: console

   nsh> bas
   bas 2.4
   Copyright 1999-2014 Michael Haardt.
   This is free software with ABSOLUTELY NO WARRANTY.
   >

Ctrl-D exits the interpreter. The test programs can be ran like this:

.. code:: console

   nsh> bastest
   Registering romdisk at /dev/ram0
   Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram0
   nsh> bas /mnt/romfs/test01.bas
    1
   hello
    0.0002
    0.0000020
    0.0000002

   nsh>

Or you can load a test into memory and execute it interactively:

.. code:: console

   nsh> bas
   bas 2.4
   Copyright 1999-2014 Michael Haardt.
   This is free software with ABSOLUTELY NO WARRANTY.
   > load "/mnt/romfs/test01.bas"
   > run
    1
   hello
    0.0002
    0.0000020
    0.0000002
   >

Configurations
==============

Each configuration is maintained in a sub-directory and can be selected as
follows:

.. code:: console

   $ ./tools/configure.sh sim:<config>

Where ``<config>`` is one of the configurations listed below.

Before building, make sure that the configuration is correct for your host
platform.

1. Linux, 32-bit CPU:

   * ``CONFIG_HOST_LINUX=y``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=y``
   * ``CONFIG_HOST_X86_64=n``
   * ``CONFIG_HOST_ARM64=n``

2. Linux, 64-bit CPU, 32-bit build:

   * ``CONFIG_HOST_LINUX=y``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=y``
   * ``CONFIG_HOST_ARM64=n``
   * ``CONFIG_SIM_X8664_MICROSOFT=n``
   * ``CONFIG_SIM_X8664_SYSTEMV=y``
   * ``CONFIG_SIM_M32=y``

3. Linux, 64-bit CPU, 64-bit build:

   * ``CONFIG_HOST_LINUX=y``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=y``
   * ``CONFIG_HOST_ARM64=n``
   * ``CONFIG_SIM_X8664_MICROSOFT=n``
   * ``CONFIG_SIM_X8664_SYSTEMV=y``
   * ``CONFIG_SIM_M32=n``

4. Cygwin, 32-bit:

   * ``CONFIG_HOST_LINUX=n``
   * ``CONFIG_HOST_WINDOWS=y``
   * ``CONFIG_WINDOWS_CYGWIN=y``
   * ``CONFIG_HOST_X86=y``
   * ``CONFIG_HOST_X86_64=n``
   * ``CONFIG_HOST_ARM64=n``

5. Cygwin64, 64-bit, 32-bit build

   I don't believe this configuration is supported by Cygwin64.

6. Cygwin64, 64-bit, 64-bit build:

   * ``CONFIG_HOST_LINUX=n``
   * ``CONFIG_HOST_WINDOWS=y``
   * ``CONFIG_WINDOWS_CYGWIN=y``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=y``
   * ``CONFIG_HOST_ARM64=n``
   * ``CONFIG_SIM_X8664_MICROSOFT=y``
   * ``CONFIG_SIM_X8664_SYSTEMV=n``
   * ``CONFIG_SIM_M32=n``

7. macOS, 64-bit, 64-bit build:

   * ``CONFIG_HOST_LINUX=n``
   * ``CONFIG_HOST_MACOS=y``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=y``
   * ``CONFIG_HOST_ARM64=n``
   * ``CONFIG_SIM_X8664_MICROSOFT=n``
   * ``CONFIG_SIM_X8664_SYSTEMV=y``
   * ``CONFIG_SIM_M32=n``

8. macOS M1, 64-bit, 64-bit build:

   * ``CONFIG_HOST_LINUX=n``
   * ``CONFIG_HOST_MACOS=y``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=n``
   * ``CONFIG_HOST_ARM64=y``
   * ``CONFIG_SIM_X8664_MICROSOFT=n``
   * ``CONFIG_SIM_X8664_SYSTEMV=y``
   * ``CONFIG_SIM_M32=n``

9. Linux ARM64, 64-bit, 64-bit build:

   * ``CONFIG_HOST_LINUX=y``
   * ``CONFIG_HOST_MACOS=n``
   * ``CONFIG_HOST_WINDOWS=n``
   * ``CONFIG_HOST_X86=n``
   * ``CONFIG_HOST_X86_64=n``
   * ``CONFIG_HOST_ARM64=y``
   * ``CONFIG_SIM_X8664_MICROSOFT=n``
   * ``CONFIG_SIM_X8664_SYSTEMV=y``
   * ``CONFIG_SIM_M32=n``

.. todo::

   Not all of the available sim configurations are documented below.

adb
---

A simple demo show how to config adb.

.. code:: console

   $ ./nuttx
   NuttShell (NSH) NuttX-10.2.0
   nsh> adbd &
   adbd [2:100]

You can use the normal adb command from host:

.. code:: console

   $ adb kill-server
   $ adb connect localhost:5555
   $ adb shell

alsa
----

This configuration enables testing audio applications on NuttX by
implementing an audio-like driver that uses ALSA to forward the audio to
the host system. It also enables the `hostfs` to enable direct access to
the host system's files mounted on the simulator. The ALSA audio driver
allows uncompressed PCM files - as well as MP3 files - to be played.

To check the audio devices:

.. code:: console

   $ ./nuttx
   NuttShell (NSH) NuttX-10.4.0
   nsh> ls /dev/audio
   /dev/audio:
   pcm0c
   pcm0p
   pcm1c
   pcm1p

* ``pcm0c`` represents the device to capture uncompressed PCM audio
* ``pcm0p`` represents the device to playback uncompressed PCM files
* ``pcm1c`` represents the device to capture MP3-encoded audio
* ``pcm1p`` represents the device to playback MP3-encoded files

**Mounting Files from Host System**

To mount files from the host system and enable them to be played in the sim:

.. code:: console

   nsh> mount -t hostfs -o fs=/path/to/audio/files/ /host
   nsh> ls /host
   /host:
   mother.mp3
   mother.wav
   .
   ..

**Playing uncompressed-PCM files**

To play uncompressed-PCM files, we can use ``nxplayer``'s ``playraw`` command. We
need 1) select the appropriate audio device to playback this file and 2) know in
advance the file's parameters (channels, bits/sample and sampling rate):

.. code:: console

   nsh> nxplayer
   NxPlayer version 1.05
   h for commands, q to exit

   nxplayer> device /dev/audio/pcm0p
   nxplayer> playraw /host/mother.wav 2 16 44100

In this example, the file ``mother.wav`` is a stereo (2-channel), 16 bits/sample
and 44,1KHz PCM-encoded file.

**Playing MP3-encoded files**

To play MP3 files, we can use ``nxplayer``'s ``play`` command directly.
We only need to select the appropriate audio device to playback this file:

.. code:: console

   nsh> nxplayer
   NxPlayer version 1.05
   h for commands, q to exit

   nxplayer> device /dev/audio/pcm1p
   nxplayer> play /host/mother.mp3

bluetooth
---------

Supports some very limited, primitive, low-level debug of the Bluetooth stack
using the Bluetooth "Swiss Army Knife" at ``apps/wireless/bluetooth/btsak`` and
the NULL Bluetooth device at ``drivers/wireless/bluetooth/bt_null.c``.

There is also support on a Linux Host for attaching the bluetooth hardware from
the host to the NuttX bluetooth stack via the HCI Socket interface over the User
Channel. This is enabled in the bthcisock configuration. In order to use this
you must give the ``nuttx`` ELF additional capabilities:

.. code:: console

   $ sudo setcap 'cap_net_raw,cap_net_admin=eip' ./nuttx

You can then monitor the HCI traffic on the host with WireShark or ``btmon``:

.. code:: console

   $ sudo btmon

configdata
----------

A unit test for the MTD configuration data driver.

cxxtest
-------

The C++ standard library test at ``apps/testing/cxxtest`` configuration. This
test is used to verify the uClibc++ port to NuttX.

.. note::

  Before you can use this example, you must first install the uClibc++ C++
  library. This is located outside of the NuttX source tree in the NuttX
  uClibc++ GIT repository. See the README.txt file there for instructions on
  how to install uClibc++

.. note::

   At present (2012/11/02), exceptions are disabled in this example
   (``CONFIG_CXX_EXCEPTION=n``). It is probably not necessary to disable
   exceptions.

.. note::

   Unfortunately, this example will not run now.

   The reason that the example will not run on the simulator has to do with when
   static constructors are enabled: In the simulator it will attempt to execute
   the static constructors before ``main()`` starts. BUT... NuttX is not
   initialized and this results in a crash.

   To really use this example, I will have to think of some way to postpone
   running C++ static initializers until NuttX has been initialized.

fb
--

A simple configuration used for some basic (non-graphic) debug of the
framebuffer character drivers using ``apps/examples/fb``.

ipforward
---------

This is an NSH configuration that includes a simple test of the NuttX IP
forwarding logic using ``apps/examples/ipforward``. That example uses two TUN
network devices to represent two networks. The test then sends packets from one
network destined for the other network. The NuttX IP forwarding logic will
recognize that the received packets are not destined for it and will forward the
logic to the other TUN network. The application logic then both sends the
packets on one network and receives and verifies the forwarded packet received
on the other network. The received packets differ from the sent packets only in
that the hop limit (TTL) has been decremented.

Be default, this test will forward TCP packets. The test can be modified to
support forwarding of ICMPv6 multicast packets with these changes to the
configuration:

.. code:: diff

   -CONFIG_EXAMPLES_IPFORWARD_TCP=y
   +CONFIG_EXAMPLES_IPFORWARD_ICMPv6=y

   +CONFIG_NET_ICMPv6=y
   +CONFIG_NET_ICMPv6_SOCKET=y
   +CONFIG_NET_ETHERNET=y
   +CONFIG_NET_IPFORWARD_BROADCAST=y

Additional required settings will also be selected when you manually select
the above via ``make menuconfig``.

loadable
--------

This configuration provides an example of loadable apps. It cannot be used with
any Windows configuration, however, because Windows does not use the ELF format.

This is the key part of the configuration:

.. code:: diff

   +CONFIG_PATH_INITIAL="/system/bin"
   +CONFIG_INIT_FILEPATH="/system/bin/nsh"

The shell is loaded from the ELF, but you can also run any of the ELFs that are
in ``/system/bin`` as they are on the ``PATH``.

minibasic
---------

This configuration was used to test the Mini Basic port at
``apps/interpreters/minibasic``.

module
------

This is a configuration to test ``CONFIG_LIBC_ELF`` with 64-bit modules. This
has ``apps/examples/module`` enabled. This configuration is intended for 64-bit
host OS.

module32
--------

This is a configuration to test ``CONFIG_LIBC_ELF`` with ``CONFIG_SIM_M32`` and
32-bit modules. This has ``apps/examples/module`` enabled. This configuration is
intended for 64-bit host OS.

mount
-----

Configures to use ``apps/examples/mount``.

mtdpart
-------

This is the ``apps/examples/mtdpart`` test using a MTD RAM driver to simulate
the FLASH part.

mtdrwb
------

This is the ``apps/examples/mtdrwb`` test using a MTD RAM driver to simulate the
FLASH part.

nettest
-------

Configures to use ``apps/examples/nettest``. This configuration enables
networking using the network TAP device.

.. note::

   As of NuttX-5.18, when built on Windows, this test does not try to use the TAP
   device (which is not available on Cygwin anyway), but inside will try to use
   the Cygwin WPCAP library.  Only the most preliminary testing has been
   performed with the Cygwin WPCAP library, however.

.. note::

   The IP address is hard-coded in ``arch/sim/src/up_wpcap.c``. You will either
   need to edit your configuration files to use 10.0.0.1 on the "target"
   (``CONFIG_EXAMPLES_NETTEST_*``) or edit ``up_wpcap.c`` to select the IP
   address that you want to use.

nimble
------

This is similar to bthcisock configuration, which uses the exposes the real
BLE stack to NuttX, but disables NuttX's own BLE stack and uses nimBLE stack
instead (built in userspace).

This configuration can be tested by running nimBLE example application "nimble"
as follows:

.. code:: console

   $ sudo setcap 'cap_net_raw,cap_net_admin=eip' nuttx
   $ ./nuttx
   NuttShell (NSH) NuttX-9.1.0
   nsh> ifup bnep0
   ifup bnep0...OK
   nsh> nimble
   hci init
   port init
   gap init
   gatt init
   ans init
   ias init
   lls init
   tps init
   hci_sock task init
   ble_host task init
   hci sock task
   host task
   advertise

At this point you should be able to detect a "nimble" BLE device when scanning
for BLE devices. You can use nRFConnect Android application from Nordic to
connect and inspect exposed GATT services.

nsh
---

Configures to use the NuttShell at ``apps/examples/nsh``. This version has one
builtin function, ``apps/examples/hello``.

.. note::

   This configuration has BINFS enabled so that the builtin applications can be
   made visible in the file system. Because of that, the builtin applications
   do not work as other examples.

   The binfs filesystem will be mounted at ``/bin`` when the system starts up:

   .. code:: console

      nsh> ls /bin
      /bin:
        hello
      nsh> echo $PATH
      /bin
      nsh> hello
      Hello, World!!
      nsh>

   Notice that the executable ``hello`` is found using the value in the ``PATH``
   variable (which was preset to `/`bin``).  If the ``PATH`` variable were not
   set then you would have to use ``/bin/hello`` on the command line.

nsh2
----

This is another example that is configured to use the NuttShell at
``apps/examples/nsh``. Like ``sim:nsh``, this version uses NSH built-in
functions. The ``nx``, ``nxhello``, and ``nxlines`` examples are included as
built-in functions.

.. note::

   X11 Configuration:

   This configuration uses an X11-based framebuffer driver.  Of course, this
   configuration can only be used in environments that support X11!  (And it
   may not even be usable in all of those environments without some
   "tweaking" See discussion below under the nx11 configuration).

   For examples, it expects to be able to include X11/Xlib.h.  That
   currently fails on my Linux box.

nx
--

Configured to use ``apps/examples/nx``.

.. note::

   Special simulated framebuffer configuration options:

   * ``CONFIG_SIM_FBHEIGHT``: Height of the framebuffer in pixels
   * ``CONFIG_SIM_FBWIDTH``: Width of the framebuffer in pixels.
   * ``CONFIG_SIM_FBBPP``: Pixel depth in bits

.. note::

   This version has NO DISPLAY and is only useful for debugging NX internals
   in environments where X11 is not supported. There is an additional
   configuration that may be added to include an X11-based simulated
   framebuffer driver:

   * ``CONFIG_SIM_X11FB``: Use X11 window for framebuffer

   See the "nx11" configuration below for more information.

nx11
----

Configures to use ``apps/examples/nx``. This configuration is similar to the nx
configuration except that it adds support for an X11-based framebuffer driver.
Of course, this configuration can only be used in environments that support X11!
(And it may not even be usable in all of those environments without some
"tweaking").

.. note::

   This configuration uses the same special simulated framebuffer
   configuration options as the nx configuration::

   * ``CONFIG_SIM_X11FB``: Use X11 window for framebuffer
   * ``CONFIG_SIM_FBHEIGHT``: Height of the framebuffer in pixels
   * ``CONFIG_SIM_FBWIDTH``: Width of the framebuffer in pixels.
   * ``CONFIG_SIM_FBBPP``: Pixel depth in bits

.. note::

   But now, since ``CONFIG_SIM_X11FB`` is also selected the following
   definitions are needed:

   * ``CONFIG_SIM_FBBPP`` (must match the resolution of the display)
   * ``CONFIG_FB_CMAP=y``

   My system has 24-bit color, but packed into 32-bit words so the correct
   setting of ``CONFIG_SIM_FBBPP`` is 32.

   For whatever value of ``CONFIG_SIM_FBBPP`` is selected, the corresponding
   ``CONFIG_NX_DISABLE_*BPP`` setting must not be disabled.

.. note::

   A X11 mouse-based touchscreen simulation can also be enabled by setting::

   * ``CONFIG_INPUT=y``
   * ``CONFIG_SIM_TOUCHSCREEN=y``

   1. If you do not have the call to sim_tcinitialize(0), the build will
      mysteriously fail claiming that it can't find up_tcenter() and
      up_tcleave().  That is a consequence of the crazy way that the
      simulation is built and can only be eliminated by calling
      up_simtouchscreen(0) from your application.

   2. You must first call up_fbinitialize(0) before calling
      up_simtouchscreen() or you will get a crash.

   3. Call sim_tcunininitializee() when you are finished with the simulated
      touchscreen.

   4. Enable CONFIG_DEBUG_INPUT=y for touchscreen debug output.

.. note::

   To get the system to compile under various X11 installations you may have
   to modify a few things. For example, in order to find libXext, I had to
   make the following change under Ubuntu 9.09:

   .. code:: console

      $ cd /usr/lib/
      $ sudo ln -s libXext.so.6.4.0 libXext.so

.. note::

   This configuration is also set up to use the ``apps/examples/nxterm`` test
   instead of ``apps/examples/nx``. To enable this configuration, First,
   select Multi-User mode as described above. Then, add the following
   definitions to the defconfig file:

   .. code:: diff

      -CONFIG_NXTERM=n
      +CONFIG_NXTERM=y

      -CONFIG_EXAMPLES_NX=y
      +CONFIG_EXAMPLES_NX=n

      -CONFIG_EXAMPLES_NXTERM=n
      +CONFIG_EXAMPLES_NXTERM=y

nxffs
-----

This is a test of the NXFFS file system using the ``apps/testing/nxffs`` test
with an MTD RAM driver to simulate the FLASH part.

nxlines
-------

This is the ``apps/examples/nxlines`` test.

nxwm
----

This is a special configuration setup for the NxWM window manager UnitTest. The
NxWM window manager can be found here at ``apps/graphics/NxWidgets/nxwm``. The
NxWM unit test can be found at ``apps/graphics/NxWidgets/UnitTests/nxwm``.

.. note::

   There is an issue with running this example under the simulation: In the
   default configuration, this example will run the NxTerm example which waits on
   ``readline()`` for console input. When it calls ``readline()``, the whole
   system blocks waiting from input from the host OS. So, in order to get this
   example to run, you must comment out the ``readline()`` call in
   ``apps/nshlib/nsh_consolemain.c`` like:

   .. code:: diff

      Index: nsh_consolemain.c
      ===================================================================
      --- nsh_consolemain.c   (revision 4681)
      +++ nsh_consolemain.c   (working copy)
      @@ -117,7 +117,8 @@
         /* Execute the startup script */

       #ifdef CONFIG_ETC_ROMFS
      -  nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
      +// REMOVE ME
      +//  nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
       #endif

         /* Then enter the command line parsing loop */
      @@ -130,7 +131,8 @@
             fflush(pstate->cn_outstream);

             /* Get the next line of input */
      -
      +sleep(2); // REMOVE ME
      +#if 0 // REMOVE ME
             ret = readline(pstate->cn_line, LINE_MAX,
                            INSTREAM(pstate), OUTSTREAM(pstate));
             if (ret > 0)
      @@ -153,6 +155,7 @@
                         "readline", NSH_ERRNO_OF(-ret));
                 nsh_exit(&pstate->cn_vtbl, 1);
               }
      +#endif // REMOVE ME
           }

         /* Clean up */


   The above workaround should no longer be necessary. However, the above
   is left in place until the solution is verified.


.. warning::

   2019-05-04

   Something has changed. Today this configuration failed to build because is
   requires ``CONFIG_NX_XYINPUT=y`` in the configuration. That indicates mouse
   or touchscreen support. Apparently, the current NxWM will not build without
   this support.

ostest
------

The "standard" NuttX ``apps/examples/ostest`` configuration.

pf_ieee802154
-------------

This is the configuration that used for unit level test of the socket support
for the PF_IEEE802154 address family. It uses the IEEE 802.15.4 loopback network
driver and the test at ``apps/examples/pf_ieee802154``.

Basic usage example:

.. code:: console

   nsh> pfserver ab:cd &
   nsh> pfclient ab:cd

pktradio
--------

This configuration is identical to the ``sixlowpan`` configuration described
below EXCEPT that it uses the generic packet radio loopback network device.

rpproxy and rpserver
--------------------

This is an example implementation for OpenAMP based on the share memory.

rpproxy:

Remote slave(client) proxy process. ``rpproxy`` creates a proxy between client and
server to allow the client to access the hardware resources on different
process.

rpserver:

Remote master (host) server process. ``rpserver`` contains all the real hardware
configuration, such as:

1. Universal Asynchronous Receiver/Transmitter (UART).
2. Specific File System.
3. Network protocol stack and real network card device.

Rpmsg driver used in this example include:

1. Rpmsg Syslog

* Redirect log to master core
  Linux kernel, NuttX, Freertos ...
* Work as early as possible
  Two phase initialization
* Never lost the log
  Hang during boot or runtime
  Full system crash(panic, watchdog ...)

2. Rpmsg TTY(UART)

* Like pseudo terminal but between two CPU
* No different from real tty(open/read/write/close)
* Full duplex communication
* Support multiple channels as need
  * Connect RTOS shell
  * Make integrated GPS like external(NMEA)
  * Make integrated modem like external(ATCMD)

3. RpmsgFS

* Like NFS but between two CPU
* Fully access remote(Linux/NuttX) File system
  * Save the tuning parameter during manufacture
  * Load the tuning parameter file in production
  * Save audio dump to file for tuning/debugging
  * Dynamic loading module from remote

4. Rpmsg Net

* Rpmsg UsrSock client
* Rpmsg UsrSock server
* Rpmsg Net driver
* Rpmsg MAC/PHY adapter

To use this example:

1. Build images

    1. Build rpserver and backup the image:

       .. code:: console

          $ ./tools/configure.sh sim:rpserver
          $ make
          $ cp nuttx ~/rpserver

    2. Distclean the build environment.

    3. Build rpproxy:

       .. code:: console

          $ ./tools/configure.sh sim:rpproxy
          $ make
          $ cp nuttx ~/rpproxy

2. Test the Rpmsg driver

    1. Rpmsg Syslog:

      Start rpserver:

      .. code:: console

         $ sudo ~/rpserver
         [    0.000000] server: SIM: Initializing

         NuttShell (NSH)
         server>

       Start rpproxy:

      .. code:: console

         $ sudo ~/rpproxy

      Check the syslog from rpproxy in rpserver terminal:

      .. code:: console

         server> [    0.000000] proxy: SIM: Initializing

    2. Rpmsg TTY(UART):

      Use cu switch the current CONSOLE to the proxy:

      .. code:: console

         server> ps
           PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK COMMAND
             0     0   0 FIFO     Kthread N-- Ready              00000000 000000 Idle Task
             1     1 224 FIFO     Kthread --- Waiting  Signal    00000000 002032 hpwork
             2     1 100 FIFO     Task    --- Running            00000000 004080 init
             3     3 224 FIFO     Kthread --- Waiting  Signal    00000002 002000 rptun proxy 0x56634fa0
         server> cu /dev/ttyproxy
         proxy> ps
           PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK COMMAND
             0     0   0 FIFO     Kthread N-- Ready              00000000 000000 Idle Task
             1     1 224 FIFO     Kthread --- Waiting  Signal    00000000 002032 hpwork
             3     3 100 FIFO     Task    --- Running            00000000 004080 init

      To switch back the console, type ``"~."`` in the cu session.

3. RpmsgFS:

   Mount the remote file system via RPMSGFS, cu to proxy first:

   .. code:: console

      server> cu
      proxy> mount -t rpmsgfs -o cpu=server,fs=/proc proc_server
      proxy> ls
      /:
        dev/
        etc/
        proc/
        proc_server/
        tmp/

   Check the uptime:

   .. code:: console

      proxy> cat proc/uptime
        833.21
      proxy> cat proc_server/uptime
        821.72

4. Rpmsg UsrSock:

   "rptun proxy" kernel thread is running:

   .. code:: console

      server> ps
        PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK COMMAND
          0     0   0 FIFO     Kthread N-- Ready              00000000 000000 Idle Task
          1     1 224 FIFO     Kthread --- Waiting  Signal    00000000 002032 hpwork
          2     1 100 FIFO     Task    --- Running            00000000 004080 init
          3     3 224 FIFO     Kthread --- Waiting  Signal    00000002 002000 rptun proxy 0x56634fa0

   Send ICMP ping to network server via rpmsg usrsock:

   .. code:: console

      server> cu
      proxy> ping 127.0.0.1
      PING 127.0.0.1 56 bytes of data
      56 bytes from 127.0.0.1: icmp_seq=0 time=20 ms
      56 bytes from 127.0.0.1: icmp_seq=1 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=2 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=3 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=4 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=5 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=6 time=20 ms
      56 bytes from 127.0.0.1: icmp_seq=7 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=8 time=10 ms
      56 bytes from 127.0.0.1: icmp_seq=9 time=10 ms
      10 packets transmitted, 10 received, 0% packet loss, time 10100 ms

Please read NETWORK-LINUX.txt if you want to try the real address.

sixlowpan
---------

This configuration was intended only for unit-level testing of the 6LoWPAN
stack. It enables networking with 6LoWPAN support and uses only a IEEE802.15.4
MAC loopback network device to supported testing.

This configuration includes ``apps/examples/nettest`` and
``apps/examples/udpblaster``. Neither are truly functional. The only intent of
this configuration is to verify that the 6LoWPAN stack correctly encodes
IEEE802.15.4 packets on output to the loopback device and correctly decodes the
returned packet.

See also the ``pktradio`` configuration.

rtptools
--------

**RTP Tools** is a set of small applications that can be used for processing RTP data.

* ``rtpplay``: playback RTP sessions recorded by ``rtpdump``
* ``rtpsend``: generate RTP packets from the textual description, generated by hand or ``rtpdump``
* ``rtpdump``: parse and print RTP packets, generating output files suitable for ``rtpplay`` and ``rtpsend``
* ``rtptrans``: RTP translator between unicast and multicast networks

This configuration is based on the :ref:`sim:tcpblaster <simulator_accessing_the_network>` and
builds the ``rtpdump``. This application is able to receive RTP packets and print the contents.
As a real-world application, one could write the received content to a FIFO and play it with
``nxplayer``.

To build it, follow the instructions for :ref:`Accessing the Network <simulator_accessing_the_network>`.

.. tip::

   One can use ``pulseaudio`` to send RTP packets through the network:

   .. code:: console

      $ pactl load-module module-null-sink sink_name=rtp format=s16le channels=2 rate=44100 sink_properties="device.description='RTP'"
      $ pactl load-module module-rtp-send source=rtp.monitor format=s16le destination_ip=10.0.1.2 port=46998

  The loaded sink ``RTP`` is used to send PC's audio to the ``10.0.1.2:46998`` address (SIM's IP).

After being able to access the network through the simulator, run:

.. code:: console

   nsh> rtpdump -F short /46998 &
   rtpdump [5:100]
   nsh> 42949704.930000 1277462397 15308
   42949704.930000 1277462714 15309

For a real-world application, check :ref:`RTP Tools on ESP32-LyraT board <esp32-lyrat_rtptools>`.

spiffs
------

This is a test of the SPIFFS file system using the ``apps/testing/fstest`` test
with an MTD RAM driver to simulate the FLASH part.

sotest
------

This is a configuration to test ``CONFIG_LIBC_ELF`` with 64-bit modules. This
has ``apps/examples/sotest`` enabled. This configuration is intended for 64-bit
host OS.

sotest32
--------

This is a configuration to test ``CONFIG_LIBC_ELF`` with ```CONFIG_SIM_M32```
and 32-bit modules. This has ``apps/examples/sotest`` enabled. This
configuration is intended for 64-bit host OS.

sqlite
-------

This configuration is used to test sqlite. Since hostfs does not support
``FIOC_FILEPATH``, it cannot currently be used in hostfs.

Basic usage example:

.. code:: console

   nsh> cd tmp
   nsh> sqlite3 test.db
   SQLite version 3.45.1 2024-01-30 16:01:20
   Enter ".help" for usage hints.
   sqlite>
   CREATE TABLE COMPANY(
     ID INT PRIMARY KEY     sqlite> (x1...> NOT NULL,
     NAME           TEXT    NOT NULL,
     AGE            (x1...> (x1...> INT     NOT NULL,
     ADDRESS        CHAR(50),
     SALARY         (x1...> (x1...> REAL
   );(x1...>
   sqlite> .quit
   sqlite>
   nsh>
   nsh> ls -l
   /tmp:
   -rwxrwxrwx       12288 test.db

tcploop
-------

This configuration performs a TCP "performance" test using
``apps/examples/tcpblaster`` and the IPv6 local loopback device. Performance is
in quotes because, while that is the intent of the tcpblaster example, this is
not an appropriate configuration for TCP performance testing. Rather, this
configuration is useful only for verifying TCP transfers over the loopback
device.

To use IPv4, modify these settings in the defconfig file:

.. code:: diff

   -# CONFIG_NET_IPv4 is not set
   -CONFIG_NET_IPv6=y
   -CONFIG_NET_IPv6_NCONF_ENTRIES=4

touchscreen
-----------

This configuration uses the simple touchscreen test at
``apps/examples/touchscreen``. This test will create an empty X11 window and
will print the touchscreen output as it is received from the simulated
touchscreen driver.

Since this example uses the simulated frame buffer driver, most of the
configuration settings discussed for the ``nx11`` configuration also apply here.
See that discussion above.

See ``apps/examples/README.txt`` for further information about build
requirements and configuration settings.

toywasm
-------

This is a configuration with toywasm.

An example usage:

.. code:: console

   NuttShell (NSH) NuttX-10.4.0
   nsh> mount -t hostfs -o fs=/tmp/wasm /mnt
   nsh> toywasm --wasi /mnt/hello.wasm
   hello
   nsh>

udgram
------

This is the same as the nsh configuration except that it includes two additional
built in applications: server and client. These applications are provided by the
test at ``apps/examples/udgram``. This configuration enables local, Unix domain
sockets and supports the test of the datagram sockets.

To use the test:

.. code:: console

   nsh> server &
   nsh> client

unionfs
-------

This is a version of NSH dedicated to performing the simple test of the Union
File System at ``apps/examples/unionfs``. The command ``unionfs`` will mount the
Union File System at ``/mnt/unionfs``. You can than compare what you see at
``/mnt/unionfs`` with the content of the ROMFS file systems at
``apps/examples/unionfs/atestdir`` and ``btestdir``.

Here is some sample output from the test:

.. code:: console

   NuttShell (NSH)
   nsh> unionfs
   Mounting ROMFS file system 1 at target=/mnt/a with source=/dev/ram4
   Mounting ROMFS file system 2 at target=/mnt/b with source=/dev/ram5
   nsh> ls /mnt/unionfs
   /mnt/unionfs:
    .
    afile.txt
    offset/

When unionfs was created, file system was joined with an offset called "offset".
Therefore, all of the file system 2 root contents will appear to reside under a
directory called ``offset/`` (although there is no directory called ``offset/``
on file system 2). File system 1 on the other hand does have an actual directory
called ``offset/``. If we list the contents of the ``offset/`` directory in the
unified file system, we see the merged contents of the file system 1 ``offset/``
directory and the file system 2 root directory:

.. code:: console

   nsh> cat /mnt/unionfs/afile.txt
   This is a file in the root directory on file system 1

   nsh> ls /mnt/unionfs/offset
   /mnt/unionfs/offset:
    afile.txt
    .
    adir/
    bfile.txt
    bdir/
   nsh> cat /mnt/unionfs/offset/afile.txt

This is a file in the ``offset/`` directory on file system 1.

.. code:: console

   nsh> cat /mnt/unionfs/offset/bfile.txt

This is another file in the root directory on file system 2.

The directory ``offset/adir`` exists on file system 1 and the directory
``adir/`` exists on file system 2. You can see that these also overlap:

.. code:: console

   nsh> ls /mnt/unionfs/offset/adir
   /mnt/unionfs/offset/adir:
    ..
    asubdir/
    adirfile.txt
    bsubdir/
    bdirfile.txt
    .

The unified directory listing is showing files from both file systems in their
respective offset ``adir/`` subdirectories. The file ``adirfile.txt`` exists in
both file system 1 and file system 2 but the version in file system 2 is
occluded by the version in file system 1. The only way that you can know which
you are looking at is by ``cat``'ing the file:

.. code:: console

   nsh> cat /mnt/unionfs/offset/adir/adirfile.txt

This is a file in directory ``offset/adir`` on file system 1.

The file on file system 1 has correctly occluded the file with the same name
on file system 2. ``bdirfile.txt``, however, only exists on file system 2, so
it is not occluded:

.. code:: console

   nsh> cat /mnt/unionfs/offset/adir/bdirfile.txt

This is another file in directory ``adir`` on file system 2.

You can see the files in the two file systems before they were unified at
``apps/examples/unionfs/atestdir`` and ``btestdir``.

userfs
------

This is another NSH configuration that includes the built-in application of
``apps/examples/userfs`` to support test of the UserFS on the simulation
platform.

To use the test:

.. code:: console

   nsh> userfs                 # Mounts the UserFS test file system at
                               # /mnt/ufstest
   nsh> mount                  # Testing is then performed by exercising the
                               # file system from the command line
   nsh> ls -l /mnt/ufstest
   nsh> cat /mnt/ufstest/File1

ustream
-------

This is the same as the nsh configuration except that it includes two addition
built in applications: server and client. These applications are provided by the
test at ``apps/examples/ustream``. This configuration enables local, Unix domain
sockets and supports the test of the stream sockets.

To use the test:

.. code:: console

   nsh> server &
   nsh> client


.. note::

   The binfs file system is mounted at ``/bin`` when the system starts up.

vncserver
---------

This a simple vnc server test configuration, Remmina is tested and recommended
since there are some compatibility issues. By default SIM will be blocked at
startup to wait client connection, if a client connected, then the fb example
will launch.

vpnkit
------

This is a configuration with VPNKit support. See NETWORK-VPNKIT.txt.

wamr
----

This is a configuration for WebAssembly sample.

1. Compile Toolchain

   1. Download WASI sdk and export the ``WASI_SDK_PATH`` path:

      .. code-block:: console

         $ wget https://github.com/WebAssembly/wasi-sdk/releases/download/wasi-sdk-19/wasi-sdk-19.0-linux.tar.gz
         $ tar xf wasi-sdk-19.0-linux.tar.gz
         # Put wasi-sdk-19.0 to your host WASI_SDK_PATH environment variable, like:
         $ export WASI_SDK_PATH=`pwd`/wasi-sdk-19.0

   2. Download Wamr "wamrc" AOT compiler and export to the ``PATH``:

    .. code-block:: console

       $ mkdir wamrc
       $ wget https://github.com/bytecodealliance/wasm-micro-runtime/releases/download/WAMR-1.1.2/wamrc-1.1.2-x86_64-ubuntu-20.04.tar.gz
       $ tar xf wamrc-1.1.2-x86_64-ubuntu-20.04.tar.gz
       $ export PATH=$PATH:$PWD

2. Configuring and running

   1. Configuring ``sim:wamr`` and compile:

      .. code-block:: console

         $ ./tools/configure.sh sim:wamr
         $ make
         ...
         Wamrc Generate AoT: /home/archer/code/nuttx/n5/apps/wasm/hello.aot
         Wamrc Generate AoT: /home/archer/code/nuttx/n5/apps/wasm/coremark.aot
         LD:  nuttx

   2. Copy the generated wasm file (Interpreter/AoT)

      .. code-block:: console

         $ cp ../apps/wasm/hello.aot .
         $ cp ../apps/wasm/hello.wasm .
         $ cp ../apps/wasm/coremark.wasm .

   3. Run iwasm

      .. code-block:: console

         $ ./nuttx
         NuttShell (NSH) NuttX-10.4.0
         nsh> iwasm /data/hello.wasm
         Hello, World!!
         nsh> iwasm /data/hello.aot
         Hello, World!!
         nsh> iwasm /data/coremark.wasm
         2K performance run parameters for coremark.
         CoreMark Size    : 666
         Total ticks      : 12000
         Total time (secs): 12.000000
         Iterations/Sec   : 5.000000
         Iterations       : 60
         Compiler version : Clang 15.0.7
         Compiler flags   : Using NuttX compilation options
         Memory location  : Defined by the NuttX configuration
         seedcrc          : 0xe9f5
         [0]crclist       : 0xe714
         [0]crcmatrix     : 0x1fd7
         [0]crcstate      : 0x8e3a
         [0]crcfinal      : 0xa14c
         Correct operation validated. See README.md for run and reporting rules.
         CoreMark 1.0 : 5.000000 / Clang 15.0.7 Using NuttX compilation options / Defined by the NuttX configuration

usbdev
------

This is a configuration with sim usbdev support.

1. Raw Gadget setup

  Get Raw Gadget code at https://github.com/xairy/raw-gadget.

  Run ``make`` in the ``raw_gadget`` and ``dummy_hcd`` directory. If
  ``raw_gadget`` build fail, you need to check which register interface meets
  your kernel version, ``usb_gadget_probe_driver`` or
  ``usb_gadget_register_driver``.

  Run ``./insmod.sh`` in the ``raw_gadget`` and ``dummy_hcd`` directory.

2. Configuration

   ``sim:usbdev`` contains two different sets of composite devices:

   * ``conn0``: ``adb`` & ``rndis``
   * ``conn1``: ``cdcacm`` & ``cdcecm``
   * ``conn2``: ``cdcncm``
   * ``conn3``: ``cdcmbim``

  You can use the ``sim:usbdev`` configuration.

3. How to run

   Run nuttx with root mode. Then, run ADB.

   .. code:: console

      $ conn 0
      $ adbd &

   Enter the ADB command on the host machine:

   .. code:: console

      $ adb kill-server
      $ adb devices
      List of devices attached
      * daemon not running; starting now at tcp:5037
      * daemon started successfully
      0101        device

   If the ADB connection fails, make sure the udev rule is added correctly.
   Edit ``/etc/udev/rules.d/51-android.rules`` file and add the following to it:

   .. code:: text

      SUBSYSTEM=="usb", ATTR{idVendor}=="1630", ATTR{idProduct}=="0042", MODE="0666", GROUP="plugdev"

   Then you can use commands such as adb shell, adb push, adb pull as normal.

   Next, run RNDIS:

   On NuttX, enter command:

   .. code:: console

      $ conn 0
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:00:00:00:00:00 at UP
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ dhcpd_start eth0
      eth0    Link encap:Ethernet HWaddr 00:00:00:00:00:00 at UP
            inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

   On the host machine, you can see the network device named ``usb0``:

   .. code:: console

      $ ifconfig
      usb0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 602
              inet 10.0.0.4  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 36:50:3d:62:b5:80  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 43  bytes 8544 (8.5 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   Then you can test the network connection using the ping command or telnet.

   Next, run CDCACM:

   On NuttX, enter the command:

   .. code:: console

      $ conn 1

   If the connection is successful, you can see ``/dev/ttyACM`` devices on both
   NuttX and host PC.

   Then you can use ``echo`` and ``cat`` command to test. On NuttX:

   .. code:: console

      nsh> echo hello > /dev/ttyACM0

    On the host machine:

    .. code:: console

       $ cat /dev/ttyACM0
       hello

   Next, run CDCECM:

   On NuttX, run:

   .. code:: console

      $ conn 1
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ dhcpd_start eth0
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
               inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

   On the host, you can see the network device named ```enx020000112233```:

   .. code:: console

      $ ifconfig
      enx020000112233: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 576
              inet 10.0.0.4  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 02:00:00:11:22:33  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 58  bytes 9143 (9.1 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   Then you can test the network connection using the ``ping`` command or ``telnet``.

   Next, run CDCNCM:

   On NuttX, run:

   .. code:: console

      $ conn 2
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 42:67:c6:69:73:51 at UP
              inet addr:10.0.1.2 DRaddr:10.0.1.1 Mask:255.255.255.0
      eth1    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ dhcpd_start eth1
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 42:67:c6:69:73:51 at UP
              inet addr:10.0.1.2 DRaddr:10.0.1.1 Mask:255.255.255.0
      eth1    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
              inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

   On the host, you can see the network device named ``enx020000112233``:

   .. code:: console

      $ ifconfig
      enx020000112233: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 576
              inet 10.0.0.2  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 02:00:00:11:22:33  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 58  bytes 9143 (9.1 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

   Then you can test the network connection using the ``ping`` command or ``telnet``.

   Next, run CDCMBIM:

   On NuttX, run:

   .. code:: console

      $ conn 3
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 42:67:c6:69:73:51 at RUNNING mtu 1500
              inet addr:10.0.1.2 DRaddr:10.0.1.1 Mask:255.255.255.0
      wwan0   Link encap:UNSPEC at RUNNING mtu 1200
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ ifconfig wwan0 10.0.0.1 netmask 255.255.255.0
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 42:67:c6:69:73:51 at RUNNING mtu 1500
              inet addr:10.0.1.2 DRaddr:10.0.1.1 Mask:255.255.255.0
      wwan0   Link encap:UNSPEC at RUNNING mtu 1200
              inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

      $ echo -n "hello from nuttx" > /dev/cdc-wdm2
      $ cat /dev/cdc-wdm2
      hello from linux

   On the host, you can see the network device named ``wwx020000112233``:

   .. code:: console

      $ sudo ifconfig wwx020000112233
      $ sudo ifconfig wwx020000112233 10.0.0.2 netmask 255.255.255.0
      $ ifconfig
      wwx020000112233: flags=4226<BROADCAST,NOARP,MULTICAST>  mtu 1500
              inet 10.0.0.2  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 02:00:00:11:22:33  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 58  bytes 9143 (9.1 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

      $ sudo cat /dev/cdc-wdm1
      hello from nuttx
      $ sudo bash -c "echo -n hello from linux > /dev/cdc-wdm1"

   Then you can test the network connection using the ``ping`` command or ``telnet``.

usbhost
-------

This is a configuration with sim usbhost support.

1. Libusb1.0 setup:

   .. code:: console

      $ sudo apt-get -y install libusb-1.0-0-dev
      $ sudo apt-get -y install libusb-1.0-0-dev:i386

2. Configuration

   ``sim:usbhost`` supports CDCACM.

   Configure the device you want to connect:

   * ``CONFIG_SIM_USB_PID=0x0042``
   * ``CONFIG_SIM_USB_VID=0x1630``

3. How to run

   Run sim usbhost with root mode, run sim usbdev or plug-in cdcacm usb device.
   Then you can use ``/dev/ttyACM`` to transfer data.

login
-----

This is a configuration with login password protection for NSH.

.. note::

   This config has password protection enabled. The login info is:

   * USERNAME: admin
   * PASSWORD: Administrator

   The encrypted password is retained in ``/etc/passwd``. I am sure that you
   will find this annoying. You can disable the password protection by
   de-selecting ``CONFIG_NSH_CONSOLE_LOGIN=y``.

can
---

This is a configuration with simulated CAN support. Both CAN character driver
and SocketCAN are enabled and use the host ``vcan0`` interface.
The ``vcan0`` host interface must be available when NuttX is started.

For the CAN character device, there is ``examples/can`` application enabled in
read-only mode.

Additionally, SocketCAN ``candump`` and ``cansend`` utils are enabled.

Below is an example of receiving CAN frames from host to NuttX.
Requirement: ``cansequence`` tool from ``linux-can/can-utils``

1. Create virtual CAN on the host:

   .. code:: console

      $ ip link add dev can0 type vcan
      $ ifconfig can0 up

2. Run NuttX:

   .. code:: console

      $ ./nuttx

3. Bring up ``can0`` on NuttX:

   .. code:: console

      nsh> ifup can0
      ifup can0...OK

4. Read CAN messages from SocketCAN on NuttX:

   .. code:: console

      nsh> candump can0

5. Send CAN messages from the host to NuttX:

   .. code:: console

      $ cansequence can0

6. Frames from the host should be received on NuttX:

   .. code:: console

      nsh> candump can0
      can0  002   [1]  00
      can0  002   [1]  01
      can0  002   [1]  02
      can0  002   [1]  03
      can0  002   [1]  04
      can0  002   [1]  05
      can0  002   [1]  06
      can0  002   [1]  07
      can0  002   [1]  08
      can0  002   [1]  09
      can0  002   [1]  0A
      can0  002   [1]  0B
      can0  002   [1]  0C
      can0  002   [1]  0D
      can0  002   [1]  0E
      can0  002   [1]  0F
      can0  002   [1]  10
      can0  002   [1]  11
      can0  002   [1]  12

ROMFS System-Init
=================

This directory contains logic to support a custom ROMFS system-init script and
start-up script. These scripts are used by by the NSH when it starts ``provided
that CONFIG_ETC_ROMFS=y``. These scripts provide a ROMFS volume that will be
mounted at ``/etc`` and will look like this at run-time:

.. code:: console

   NuttShell (NSH) NuttX-12.10.0
   nsh> ls -Rl /etc
   /etc:
    dr-xr-xr-x       0 .
    -r--r--r--      20 group
    dr-xr-xr-x       0 init.d/
    -r--r--r--      35 passwd
   /etc/init.d:
    dr-xr-xr-x       0 ..
    -r--r--r--     110 rcS
    -r--r--r--     110 rc.sysinit
   nsh>

``/etc/init.d/rc.sysinit`` is system init script; ``/etc/init.d/rcS`` is the
start-up script; ``/etc/passwd`` is a the password file. It supports a single
user:

.. code:: text

   USERNAME:  admin
   PASSWORD:  Administrator

.. code:: console

   nsh> cat /etc/passwd
   admin:8Tv+Hbmr3pLVb5HHZgd26D:0:0:/

The encrypted passwords in the provided passwd file are only valid if the
TEA key is set to: 012345678 9abcdef0 012345678 9abcdef0.

Changes to either the key or the password word will require regeneration of the
``nsh_romfimg.h`` header file.

The format of the password file is:

.. code:: text

   user:x:uid:gid:home

Where:

* user: User name
* x: Encrypted password
* uid: User ID (0 for now)
* gid: Group ID (0 for now)
* home: Login directory (/ for now)

``/etc/group`` is a group file. It is not currently used.

.. code:: console

   nsh> cat /etc/group
   root:*:0:root,admin

The format of the group file is:

.. code:: text

   group:x:gid:users

Where:

* group: The group name
* x: Group password
* gid: Group ID
* users: A comma separated list of members of the group

Updating the ROMFS File System
------------------------------

The content on the ``nsh_romfsimg.h`` header file is generated from a sample
directory structure. You can directly modify files under ``etc/`` folder, The
build system will regenerate ``nsh_romfsimg.h`` automatically.

See the ``sim:nsh`` configuration for an example of the use of this file system.

Replacing the Password File
---------------------------

The ``sim:nsh`` configuration can also be used to create a new password file.

First, make these configuration changes:

1. Disable logins

   .. code:: diff

      - CONFIG_NSH_CONSOLE_LOGIN=y
      + # CONFIG_NSH_CONSOLE_LOGIN is not set
        # CONFIG_NSH_TELNET_LOGIN is not set

2. Move the password file to a write-able file system:

   .. code:: diff

      - CONFIG_FSUTILS_PASSWD_PATH="/etc/passwd"
      + CONFIG_FSUTILS_PASSWD_PATH="/tmp/passwd"

3. Make the password file modifiable

   .. code:: diff

      - CONFIG_FSUTILS_PASSWD_READONLY=y
      # CONFIG_FSUTILS_PASSWD_READONLY is not set

Now rebuild the simulation. No login should be required to enter the shell and
you should find the ``useradd``, ``userdel``, and ``passwd`` commands available
in the help summary, provided that they are enabled. Make certain that the
``useradd`` command is not disabled:

.. code:: text

   # CONFIG_NSH_DISABLE_USERADD is not set

Use the NSH ``useradd`` command to add new uses with new user passwords like:

.. code:: console

   nsh> useradd <username> <password>

Do this as many times as you would like. Each time that you do this a new entry
with an encrypted password will be added to the ``passwd`` file at
``/tmp/passwd``. You can see the ``passwd`` file like:

.. code:: console

   nsh> cat /tmp/passwd

When you are finished, you can simply copy the ``/tmp/passwd`` content from the
``cat`` command and paste it into an editor. Make sure to remove any carriage
returns that may have ended up on the file if you are using Windows.

Then recreate the ``nsh_romfsimg.h`` file as described above. In step 2, simply
replace the old ``/etc/passwd`` file with the one in your editor. When you are
finished, the new passwd file will be in the ROMFS file system at the path
``/etc/passwd``. When you restore the original NSH sim configuration, these are
the passwords that will be used.
