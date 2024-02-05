===
SIM
===

Overview
========

Description
-----------

This README file describes the contents of the build configurations available
for the NuttX "sim" target.  The sim target is a NuttX port that runs as a
user-space program under Linux, Cygwin, or macOS.  It is a very "low fidelity"
embedded system simulation:  This environment does not support any kind of
asynchronous events -- there are nothing like interrupts in this context.
Therefore, there can be no pre-empting events.

Fake Interrupts
---------------

In order to get timed behavior, the system timer "interrupt handler" is called
from the sim target's IDLE loop.  The IDLE runs whenever there is no other
task running.  So, for example, if a task calls sleep(), then that task will
suspend wanting for the time to elapse.  If nothing else is available to run,
then the IDLE loop runs and the timer increments, eventually re-awakening the
sleeping task.

Context switching is based on logic similar to setjmp() and longjmp().

The sim target is used primarily as a development and test platform for new
RTOS features.  It is also of academic interest.  But it has no real-world
application that I know of.

Timing Fidelity
---------------

NOTE: The sim target's IDLE loop to delay on each call so that the system
"timer interrupt" is called at a rate approximately correct for the system
timer tick rate.  This option can be enabled with CONFIG_SIM_WALLTIME_SIGNAL
which will drive the entire simulation by using a host timer that ticks at
CONFIG_USEC_PER_TICK.  This option will no longer deliver 'tick' events
from Idle task and it will generate them from the host signal handler.
Another option is to use CONFIG_SIM_WALLTIME_SLEEP which will enable the
tick events to be delayed from the Idle task by using a host sleep call.

Debugging
=========

One of the best reasons to use the simulation is that is supports great Linux-
based debugging.  Here are the steps that I following to use the Linux ddd
graphical front-end to GDB:

1. Modify the top-level configuration file.  Enable debug symbols by defining
   the following::

       cd <NuttX-Directory>
       CONFIG_DEBUG_SYMBOLS=y

2. Re-build::

       cd <NuttX-Directory>
       make clean
       make

3. Then start the debugging::

       ddd nuttx &
       gdb> b user_start
       gdb> r

NOTE:  This above steps work fine on Linux, Cygwin, and macOS.
On Cygwin, you will need to start the Cygwin-X server before running ddd.
On macOS, it's probably easier to use lldb instead of gdb.

Issues
======

64-Bit Issues
-------------

As mentioned above, context switching is based on logic like setjmp() and
longjmp().  This context switching is available for 32-bit and 64-bit targets.
You must, however, set the correct target in the configuration before you
build: CONFIG_HOST_X86_64 or CONFIG_HOST_X86 for 64- and 32-bit targets,
respectively.  On a 64-bit machine, you can also force the 32-bit build with
CONFIG_SIM_M32=y (which does not seem to be supported by more contemporary
x86_64 compilers).

There are other 64-bit issues as well.  For example, addresses are retained in
32-bit unsigned integer types in a few places.  On a 64-bit machine, the 32-
bit address storage may corrupt 64-bit addressing.  NOTE:  This is really a
bug -- addresses should not be retained in uint32_t types but rather in
uintptr_t types to avoid issues just like this.

Compiler differences
--------------------

operator new:

  Problem:     "'operator new' takes size_t ('...') as first parameter"

  Workaround:   Add -fpermissive to the compilation flags

Stack Size Issues
-----------------

When you run the NuttX simulation, it uses stacks allocated by NuttX from the
NuttX heap.  The memory management model is exactly the same in the simulation
as in a real, target system.  This is good because this produces a higher
fidelity simulation.

However, when the simulation calls into the host OS libraries, it will still
use these small simulation stacks.  This happens, for example, when you call
into the system to get and put characters to the console window or when you
make X11 calls into the system.  The programming model within those libraries
will assume the host OS environment where the stack size grows dynamically
and not the small, limited stacks of a deeply embedded system.

As a consequence, those system libraries may allocate large data structures on
the stack and overflow the small NuttX stacks.  X11, in particular, requires
large stacks.  If you are using X11 in the simulation, make sure that you set
aside a "lot" of stack for the X11 library calls (maybe 8 or 16Kb). The stack
size for the thread that begins with user start is controlled by the
configuration setting CONFIG_INIT_STACKSIZE; you may need to increase this
value to larger number to survive the X11 library calls.

If you are running X11 applications such as NSH add-on programs, then the
stack size of the add-on program is controlled in another way.  Here are the
steps for increasing the stack size in that case::

  cd ../apps/builtin    # Go to the builtin apps directory
  vi builtin_list.h     # Edit this file and increase the stack size of the add-on
  rm .built *.o         # This will force the builtin apps logic to rebuild

Symbol Collisions
-----------------

The simulation build is a two pass build:

  1. On the first pass, an intermediate, partially relocatable object is
     created called nuttx.rel.  This includes all of the files that are part
     of the NuttX "domain."

  2. On the second pass, the files which are in the host OS domain are built
     and then linked with nuttx.rel to generate the simulation program.

NuttX is a POSIX compliant RTOS and is normally built on a POSIX compliant
host environment (like Linux, Cygwin, or macOS).  As a result, the same
symbols are exported by both the NuttX domain and the host domain.  How can
we keep them separate?

This is done using the special file nuttx-name.dat.  This file just contains a
mapping of original function names to new function names.  For example, the
NuttX printf() will get the new name NXprintf().

This nuttx-names.dat file is used by the objcopy program between pass1 and
pass2 to rename all of the symbols in the nuttx.rel object so that they do not
collide with names provided by the host OS in the host PC domain.

Occasionally, as you test new functionality, you will find that you need to
add more names to the nuttx-names.dat file.  If there is a missing name
mapping in nuttx-names.dat, the symptoms may be very obscure and difficult to
debug.  What happens in this case is that when logic in nuttx.rel intended to
call the NuttX domain function, it instead calls into the host OS function of
the same name.

Often you can survive such events.  For example, it really should not matter
which version of strlen() you call.  Other times, it can cause subtle,
mysterious errors.  Usually, however, callng the wrong function in the wrong
OS results in a fatal crash.

On macOS, instead of objcopy, -unexported_symbols_list linker option is used
to hide symbols in the NuttX domain, using the same list of symbols from
nuttx-name.dat.

Networking Issues
-----------------

I never did get networking to work on the sim target.  It tries to use the tap
device (/dev/net/tun) to emulate an Ethernet NIC, but I never got it correctly
integrated with the NuttX networking. (I probably should try using raw sockets
instead.)

Update:  Max Holtzberg reports to me that the tap device actually does work
properly, but not in an NSH configuration because stdio operations freeze the
simulation.

REVISIT: This may not long be an issue even with NSH because of the recent
redesign of how the stdio devices are handled in the simulation (they should
no longer freeze the simulation).

Update: Please issue these commands to setup the reliable network on Ubuntu::

  sudo apt-get -y install net-tools
  sudo nuttx/tools/simbridge.sh eth0 on

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

For example, on Ubuntu 9.09, I had to do the following to get a clean build::

    cd /usr/lib/
    sudo ln -s libXext.so.6.4.0 libXext.so

(I also get a segmentation fault at the conclusion of the NX test -- that will
need to get looked into as well.)

The X11 examples builds on Cygwin, but does not run.  The last time I tried
it, XOpenDisplay() aborted the program.  UPDATE:  This was caused by the small
stack size and can be fixed by increasing the size of the NuttX stack that
calls into X11.  See the discussion "Stack Size Issues" above.

Update: You may need issue this command with the latest Ubuntu before launch::

  sudo xhost +

Cygwin64 Issues
---------------
There are some additional issues using the simulator with Cygwin64.  Below is
the summary of the changes that I had to make to get the simulator working in
that environment:

  CONFIG_HOST_X86_64=y, CONFIG_SIM_M32=n

    Need to select X64_64.  Cygwin64 tools do not seem to support any option
    to build a 32-bit target.

  CONFIG_SIM_CYGWIN_DECORATED=n

    Older versions of Cygwin tools decorated C symbol names by adding an
    underscore to the beginning of the symbol name.  Newer versions of Cygwin
    do not seem to do this.  Deselecting CONFIG_SIM_CYGWIN_DECORATED will
    select the symbols without the leading underscore as needed by the
    Cygwin64 toolchain.

    How do you know if you need this option?  You could look at the generated
    symbol tables to see if there are underscore characters at the beginning
    of the symbol names.  Or, if you need this option, the simulation will not
    run:  It will crash early, probably in some function due to the failure to
    allocate memory.

    In this case, when I tried to run nutt.exe from the command line, it
    exited silently.  Running with GDB I get the following (before hitting a
    breakpoint at main())::

      (gdb) r
      Starting program: /cygdrive/c/Users/Gregory/Documents/projects/nuttx/master/nuttx/nuttx.exe
      [New Thread 6512.0xda8]
      [New Thread 6512.0x998]
            1 [main] nuttx 6512 C:\Users\Gregory\Documents\projects\nuttx\master\nuttx\nuttx.exe: *** fatal error - Internal error: Out of memory for new path buf.
          736 [main] nuttx 6512 cygwin_exception::open_stackdumpfile: Dumping stack trace to nuttx.exe.stackdump
      [Thread 6512.0x998 exited with code 256]
      [Inferior 1 (process 6512) exited with code 0400]

  CONFIG_SIM_X8664_SYSTEMV=n, CONFIG_SIM_X8664_MICROSOFT=y

    Select Microsoft x64 calling convention.

    The Microsoft x64 calling convention is followed on Microsoft Windows and
    pre-boot UEFI (for long mode on x86-64). It uses registers RCX, RDX, R8,
    R9 for the first four integer or pointer arguments (in that order), and
    XMM0, XMM1, XMM2, XMM3 are used for floating point arguments. Additional
    arguments are pushed onto the stack (right to left). Integer return values
    (similar to x86) are returned in RAX if 64 bits or less. Floating point
    return values are returned in XMM0. Parameters less than 64 bits long are
    not zero extended; the high bits are not zeroed.

SMP
---

  This configuration has basic support for SMP testing.  The simulation
  supports the emulation of multiple CPUs by creating multiple pthreads, each
  running a copy of the simulation in the same process address space.

  At present, the SMP simulation is not fully functional:  It does operate on
  the simulated CPU threads for a few context switches then fails during a
  setjmp() operation.  I suspect that this is not an issue with the NuttX SMP
  logic but more likely some chaos in the pthread controls. I have seen
  similar such strange behavior other times that I have tried to use
  setjmp/longmp from a signal handler! Like when I tried to implement
  simulated interrupts using signals.

  Apparently, if longjmp is invoked from the context of a signal handler, the
  result is undefined:
  http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1318.htm

  Update: The dead lock is due to up_testset call pthread API for synchronization
  inside the signal handler. After switching to atomic API, the problem get resolved.

  You can enable SMP for ostest configuration by enabling::

    +CONFIG_SPINLOCK=y
    +CONFIG_SMP=y
    +CONFIG_SMP_NCPUS=2

  And you can enable some additional debug output with::

    -# CONFIG_DEBUG_SCHED is not set
    +CONFIG_DEBUG_SCHED=y

    -# CONFIG_SCHED_INSTRUMENTATION is not set
    -# CONFIG_SCHED_INSTRUMENTATION_SWITCH is not set
    +CONFIG_SCHED_INSTRUMENTATION=y
    +CONFIG_SCHED_INSTRUMENTATION_SWITCH=y

  The SMP configuration will run with::

    CONFIG_SMP_NCPUS=1

  In this case there is, of course, no multi-CPU processing, but this does
  verify the correctness of some of the basic SMP logic.

  The NSH configuration can also be forced to run SMP, but suffers from the
  same quirky behavior.  It can be made reliable if you modify
  arch/sim/src/up_idle.c so that the IDLE loop only runs for CPU0. Otherwise,
  often simuart_post() will be called from CPU1 and it will try to restart NSH
  on CPU0 and, again, the same quirkiness occurs.

  Update: Only CPU0 call up_idle now, other CPUs have a simple idle loop::

    /* The idle Loop */

    for (; ; )
      {
        /* Give other pthreads/CPUs a shot */

        pthread_yield();
      }

  So it isn't a problem any more.

  But for example, this command::

    nsh> sleep 1 &

  will execute the sleep command on CPU1 which has worked every time that I
  have tried it (which is not too many times).

BASIC
=====

  I have used the sim/nsh configuration to test Michael Haardt's BASIC
  interpreter that you can find at apps/interpreters/bas.

    Bas is an interpreter for the classic dialect of the programming language
    BASIC.  It is pretty compatible to typical BASIC interpreters of the
    1980s, unlike some other UNIX BASIC interpreters, that implement a
    different syntax, breaking compatibility to existing programs.  Bas offers
    many ANSI BASIC statements for structured programming, such as procedures,
    local variables and various loop types.  Further there are matrix
    operations, automatic LIST indentation and many statements and functions
    found in specific classic dialects.  Line numbers are not required.

  There is also a test suite for the interpreter that can be found at
  apps/examples/bastest.

Configuration
-------------
  Below are the recommended configuration changes to use BAS with the
  stm32f4discovery/nsh configuration:

  Dependencies::

    CONFIG_LIBC_EXECFUNCS=y      : exec*() functions are required
    CONFIG_LIBM=y                : Some floating point library is required
    CONFIG_LIBC_FLOATINGPOINT=y  : Floating point printing support is required
    CONFIG_LIBC_TMPDIR="/tmp"    : Writeable temporary files needed for some commands

  Enable the BASIC interpreter.  Other default options should be okay::

    CONFIG_INTERPRETERS_BAS=y    : Enables the interpreter
    CONFIG_INTERPRETER_BAS_VT100=y

  The BASIC test suite can be included::

     CONFIG_FS_ROMFS=y           : ROMFS support is needed
     CONFIG_EXAMPLES_BASTEST=y   : Enables the BASIC test setup
     CONFIG_EXAMPLES_BASTEST_DEVMINOR=6
     CONFIG_EXAMPLES_BASTEST_DEVPATH="/dev/ram6"

Usage
-----
  This setup will initialize the BASIC test (optional):  This will mount a
  ROMFS file system at /mnt/romfs that contains the BASIC test files::

      nsh> bastest
      Registering romdisk at /dev/ram6
      Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram6
      nsh>

  The interactive interpreter is started like::

      nsh> bas
      bas 2.4
      Copyright 1999-2014 Michael Haardt.
      This is free software with ABSOLUTELY NO WARRANTY.
      >

      Ctrl-D exits the interpreter.

      The test programs can be ran like this:

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

  Or you can load a test into memory and execute it interactively::

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

Common Configuration Information
================================

  1. Each configuration is maintained in a sub-directory and can be selected
     as follow::

       tools/configure.sh sim:<subdir>

     Where <subdir> is one of the following sub-directories.

  2. All configurations uses the mconf-based configuration tool.  To change
     this configuration using that tool, you should:

     a. Build and install the kconfig mconf tool.  See nuttx/README.txt and
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. Before building, make sure that the configuration is correct for your
     host platform:

     a. Linux, 32-bit CPU::

            CONFIG_HOST_LINUX=y
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=y
            CONFIG_HOST_X86_64=n
            CONFIG_HOST_ARM64=n

     b. Linux, 64-bit CPU, 32-bit build::

            CONFIG_HOST_LINUX=y
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=y
            CONFIG_HOST_ARM64=n
            CONFIG_SIM_X8664_MICROSOFT=n
            CONFIG_SIM_X8664_SYSTEMV=y
            CONFIG_SIM_M32=y

     c. Linux, 64-bit CPU, 64-bit build::

            CONFIG_HOST_LINUX=y
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=y
            CONFIG_HOST_ARM64=n
            CONFIG_SIM_X8664_MICROSOFT=n
            CONFIG_SIM_X8664_SYSTEMV=y
            CONFIG_SIM_M32=n

     d. Cygwin, 32-bit::

            CONFIG_HOST_LINUX=n
            CONFIG_HOST_WINDOWS=y
            CONFIG_WINDOWS_CYGWIN=y
            CONFIG_HOST_X86=y
            CONFIG_HOST_X86_64=n
            CONFIG_HOST_ARM64=n

     e. Cygwin64, 64-bit, 32-bit build

        I don't believe this configuration is supported by Cygwin64

     f. Cygwin64, 64-bit, 64-bit build::

            CONFIG_HOST_LINUX=n
            CONFIG_HOST_WINDOWS=y
            CONFIG_WINDOWS_CYGWIN=y
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=y
            CONFIG_HOST_ARM64=n
            CONFIG_SIM_X8664_MICROSOFT=y
            CONFIG_SIM_X8664_SYSTEMV=n
            CONFIG_SIM_M32=n

     g. macOS, 64-bit, 64-bit build::

            CONFIG_HOST_LINUX=n
            CONFIG_HOST_MACOS=y
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=y
            CONFIG_HOST_ARM64=n
            CONFIG_SIM_X8664_MICROSOFT=n
            CONFIG_SIM_X8664_SYSTEMV=y
            CONFIG_SIM_M32=n

     h. macOS M1, 64-bit, 64-bit build::

            CONFIG_HOST_LINUX=n
            CONFIG_HOST_MACOS=y
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=n
            CONFIG_HOST_ARM64=y
            CONFIG_SIM_X8664_MICROSOFT=n
            CONFIG_SIM_X8664_SYSTEMV=y
            CONFIG_SIM_M32=n

     i. Linux ARM64, 64-bit, 64-bit build::

            CONFIG_HOST_LINUX=y
            CONFIG_HOST_MACOS=n
            CONFIG_HOST_WINDOWS=n
            CONFIG_HOST_X86=n
            CONFIG_HOST_X86_64=n
            CONFIG_HOST_ARM64=y
            CONFIG_SIM_X8664_MICROSOFT=n
            CONFIG_SIM_X8664_SYSTEMV=y
            CONFIG_SIM_M32=n

Configurations
==============

adb
---

A simple demo show how to config adb::

    $ ./nuttx
    NuttShell (NSH) NuttX-10.2.0
    nsh> adbd &
    adbd [2:100]

You can use the normal adb command from host::

    adb kill-server
    adb connect localhost:5555
    adb shell

alsa
----

This configuration enables testing audio applications on NuttX by
implementing an audio-like driver that uses ALSA to forward the audio to
the host system. It also enables the `hostfs` to enable direct access to
the host system's files mounted on the simulator. The ALSA audio driver
allows uncompressed PCM files - as well as MP3 files - to be played.

To check the audio devices::

    $ ./nuttx
    NuttShell (NSH) NuttX-10.4.0
    nsh> ls /dev/audio
    /dev/audio:
    pcm0c
    pcm0p
    pcm1c
    pcm1p

- `pcm0c` represents the device to capture uncompressed PCM audio;
- `pcm0p` represents the device to playback uncompressed PCM files;
- `pcm1c` represents the device to capture MP3-encoded audio;
- `pcm1p` represents the device to playback MP3-encoded files;

Mounting Files from Host System
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To mount files from the host system and enable them to be played in the sim::

    nsh> mount -t hostfs -o fs=/path/to/audio/files/ /host
    nsh> ls /host
    /host:
    mother.mp3
    mother.wav
    .
    ..

Playing uncompressed-PCM files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To play uncompressed-PCM files, we can use `nxplayer`'s `playraw` command.
We need 1) select the appropriate audio device to playback this file and
1) know in advance the file's parameters (channels, bits/sample and
sampling rate)::

    nsh> nxplayer
    NxPlayer version 1.05
    h for commands, q to exit

    nxplayer> device /dev/audio/pcm0p
    nxplayer> playraw /host/mother.wav 2 16 44100

In this example, the file `mother.wav` is a stereo (2-channel),
16 bits/sample and 44,1KHz PCM-encoded file.

Playing MP3-encoded files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To play MP3 files, we can use `nxplayer`'s `play` command directly.
We only need to select the appropriate audio device to playback this file::

    nsh> nxplayer
    NxPlayer version 1.05
    h for commands, q to exit

    nxplayer> device /dev/audio/pcm1p
    nxplayer> play /host/mother.mp3

bluetooth
---------

Supports some very limited, primitive, low-level debug of the Bluetooth
stack using the Bluetooth "Swiss Army Knife" at
apps/wireless/bluetooth/btsak and the NULL Bluetooth device at
drivers/wireless/bluetooth/bt_null.c

There is also support on a Linux Host for attaching the bluetooth hardware
from the host to the NuttX bluetooth stack via the HCI Socket interface
over the User Channel.  This is enabled in the bthcisock configuration.
In order to use this you must give the nuttx elf additional capabilities::

    sudo setcap 'cap_net_raw,cap_net_admin=eip' ./nuttx

You can then monitor the HCI traffic on the host with wireshark or btmon::

    sudo btmon

configdata
----------

A unit test for the MTD configuration data driver.

cxxtest
-------

The C++ standard library test at apps/testing/cxxtest configuration.  This
test is used to verify the uClibc++ port to NuttX.

NOTES

  1. Before you can use this example, you must first install the uClibc++ C++
     library.  This is located outside of the NuttX source tree in the NuttX
     uClibc++ GIT repository.  See the README.txt file there for instructions
     on how to install uClibc++

  2. At present (2012/11/02), exceptions are disabled in this example
     (CONFIG_CXX_EXCEPTION=n).  It is probably not necessary to disable
     exceptions.

  3. Unfortunately, this example will not run now.

     The reason that the example will not run on the simulator has to do with
     when static constructors are enabled:  In the simulator it will attempt
     to execute the static constructors before main() starts. BUT... NuttX is
     not initialized and this results in a crash.

     To really use this example, I will have to think of some way to postpone
     running C++ static initializers until NuttX has been initialized.

fb
--

A simple configuration used for some basic (non-graphic) debug of the
framebuffer character drivers using apps/examples/fb.

ipforward
---------

This is an NSH configuration that includes a simple test of the NuttX IP
forwarding logic using apps/examples/ipforward.  That example uses two TUN
network devices to represent two networks.  The test then sends packets from
one network destined for the other network.  The NuttX IP forwarding logic
will recognize that the received packets are not destined for it and will
forward the logic to the other TUN network.  The application logic then both
sends the packets on one network and receives and verifies the forwarded
packet received on the other network.  The received packets differ from the
sent packets only in that the hop limit (TTL) has been decremented.

Be default, this test will forward TCP packets.  The test can be modified to
support forwarding of ICMPv6 multicast packets with these changes to the
.config file::

    -CONFIG_EXAMPLES_IPFORWARD_TCP=y
    +CONFIG_EXAMPLES_IPFORWARD_ICMPv6=y

    +CONFIG_NET_ICMPv6=y
    +CONFIG_NET_ICMPv6_SOCKET=y
    +CONFIG_NET_ETHERNET=y
    +CONFIG_NET_IPFORWARD_BROADCAST=y

Additional required settings will also be selected when you manually select
the above via 'make menuconfig'.

loadable
--------

This configuration provides an example of loadable apps.  It cannot be used
with any Windows configuration, however, because Windows does not use the
ELF format.

This is the key part of the configuration::

      +CONFIG_PATH_INITIAL="/system/bin"
      +CONFIG_INIT_FILEPATH="/system/bin/nsh"

The shell is loaded from the elf, but you can also run any of the ELFs that are in /system/bin as they are on the "PATH"

minibasic
---------

This configuration was used to test the Mini Basic port at
apps/interpreters/minibasic.

module
------

This is a configuration to test CONFIG_LIBC_MODLIB with 64-bit modules.
This has apps/examples/module enabled.
This configuration is intended for 64-bit host OS.

module32
--------

This is a configuration to test CONFIG_LIBC_MODLIB with CONFIG_SIM_M32
and 32-bit modules.
This has apps/examples/module enabled.
This configuration is intended for 64-bit host OS.

mount
-----

Configures to use apps/examples/mount.

mtdpart
-------

This is the apps/examples/mtdpart test using a MTD RAM driver to
simulate the FLASH part.

mtdrwb
------

This is the apps/examples/mtdrwb test using a MTD RAM driver to
simulate the FLASH part.

nettest
-------

Configures to use apps/examples/nettest.  This configuration enables
networking using the network TAP device.

NOTES:

  1. The NuttX network is not, however, functional on the Linux TAP device
     yet.

     UPDATE:  The TAP device does apparently work according to a NuttX user
     (provided that it is not used with NSH: NSH waits on readline() for
     console input.  When it calls readline(), the whole system blocks waiting
     from input from the host OS).  My failure to get the TAP device working
     appears to have been a cockpit error.

  2. As of NuttX-5.18, when built on Windows, this test does not try to use
     the TAP device (which is not available on Cygwin anyway), but inside will
     try to use the Cygwin WPCAP library.  Only the most preliminary testing
     has been performed with the Cygwin WPCAP library, however.

     NOTE that the IP address is hard-coded in arch/sim/src/up_wpcap.c. You
     will either need to edit your configuration files to use 10.0.0.1 on the
     "target" (CONFIG_EXAMPLES_NETTEST_*) or edit up_wpcap.c to select the IP
     address that you want to use.

nimble
------

This is similar to bthcisock configuration, which uses the exposes the real
BLE stack to NuttX, but disables NuttX's own BLE stack and uses nimBLE stack
instead (built in userspace).

This configuration can be tested by running nimBLE example application "nimble"
as follows::

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
for BLE devices. You can use nRFConnect Android application from Nordic to connect
and inspect exposed GATT services.

nsh
---

Configures to use the NuttShell at apps/examples/nsh.

NOTES:

  1. This version has one builtin function:  This configuration::

           apps/examples/hello.

  2. This version has password protection enabled.  Here is the login info::

           USERNAME:  admin
           PASSWORD:  Administrator

     The encrypted password is retained in /etc/passwd.  I am sure that
     you will find this annoying.  You can disable the password protection
     by de-selecting CONFIG_NSH_CONSOLE_LOGIN=y.

  3. This configuration has BINFS enabled so that the builtin applications can
     be made visible in the file system.  Because of that, the builtin
     applications do not work as other examples.

     The binfs filesystem will be mounted at /bin when the system starts up::

           nsh> ls /bin
           /bin:
             hello
           nsh> echo $PATH
           /bin
           nsh> hello
           Hello, World!!
           nsh>

     Notice that the executable 'hello' is found using the value in the PATH
     variable (which was preset to "/bin").  If the PATH variable were not set
     then you would have to use /bin/hello on the command line.

nsh2
----

This is another example that configures to use the NuttShell at
apps/examples/nsh. Like nsh, this version uses NSH built-in functions:  The
nx, nxhello, and nxlines examples are included as built-in functions.

NOTES:

  1. X11 Configuration

     This configuration uses an X11-based framebuffer driver.  Of course, this
     configuration can only be used in environments that support X11!  (And it
     may not even be usable in all of those environments without some
     "tweaking" See discussion below under the nx11 configuration).

     For examples, it expects to be able to include X11/Xlib.h.  That
     currently fails on my Linux box.

nx
--

Configures to use apps/examples/nx.

NOTES:

  1. Special Framebuffer Configuration

     Special simulated framebuffer configuration options::

           CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
           CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
           CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. No Display!

     This version has NO DISPLAY and is only useful for debugging NX internals
     in environments where X11 is not supported.  There is an additional
     configuration that may be added to include an X11-based simulated
     framebuffer driver::

           CONFIG_SIM_X11FB    - Use X11 window for framebuffer

     See the "nx11" configuration below for more information.

nx11
----

Configures to use apps/examples/nx.  This configuration is similar to the nx
configuration except that it adds support for an X11-based framebuffer
driver.  Of course, this configuration can only be used in environments that
support X11!  (And it may not even be usable in all of those environments
without some "tweaking").

  1. Special Framebuffer Configuration

     This configuration uses the same special simulated framebuffer
     configuration options as the nx configuration::

           CONFIG_SIM_X11FB    - Use X11 window for framebuffer
           CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
           CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
           CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. X11 Configuration

     But now, since CONFIG_SIM_X11FB is also selected the following
     definitions are needed::

           CONFIG_SIM_FBBPP (must match the resolution of the display).
           CONFIG_FB_CMAP=y

     My system has 24-bit color, but packed into 32-bit words so the correct
     setting of CONFIG_SIM_FBBPP is 32.

     For whatever value of CONFIG_SIM_FBBPP is selected, the corresponding
     CONFIG_NX_DISABLE_*BPP setting must not be disabled.

  3. Touchscreen Support

     A X11 mouse-based touchscreen simulation can also be enabled by setting::

           CONFIG_INPUT=y
           CONFIG_SIM_TOUCHSCREEN=y

     NOTES:

     a. If you do not have the call to sim_tcinitialize(0), the build will
        mysteriously fail claiming that it can't find up_tcenter() and
        up_tcleave().  That is a consequence of the crazy way that the
        simulation is built and can only be eliminated by calling
        up_simtouchscreen(0) from your application.

     b. You must first call up_fbinitialize(0) before calling
        up_simtouchscreen() or you will get a crash.

     c. Call sim_tcunininitializee() when you are finished with the simulated
        touchscreen.

     d. Enable CONFIG_DEBUG_INPUT=y for touchscreen debug output.

  4. X11 Build Issues

     To get the system to compile under various X11 installations you may have
     to modify a few things.  For example, in order to find libXext, I had to
     make the following change under Ubuntu 9.09::

           cd /usr/lib/
           sudo ln -s libXext.so.6.4.0 libXext.so

  5. apps/examples/nxterm

     This configuration is also set up to use the apps/examples/nxterm test
     instead of apps/examples/nx.  To enable this configuration, First,
     select Multi-User mode as described above.  Then, add the following
     definitions to the defconfig file::

           -CONFIG_NXTERM=n
           +CONFIG_NXTERM=y

           -CONFIG_EXAMPLES_NX=y
           +CONFIG_EXAMPLES_NX=n

           -CONFIG_EXAMPLES_NXTERM=n
           +CONFIG_EXAMPLES_NXTERM=y

     See apps/examples/README.txt for further details.

nxffs
-----

This is a test of the NXFFS file system using the apps/testing/nxffs test
with an MTD RAM driver to simulate the FLASH part.

nxlines
-------

This is the apps/examples/nxlines test.

nxwm
----

This is a special configuration setup for the NxWM window manager UnitTest.
The NxWM window manager can be found here::

    apps/graphics/NxWidgets/nxwm

The NxWM unit test can be found at::

    apps/graphics/NxWidgets/UnitTests/nxwm

NOTES

  1. There is an issue with running this example under the simulation:  In the
     default configuration, this example will run the NxTerm example which
     waits on readline() for console input.  When it calls readline(), the
     whole system blocks waiting from input from the host OS.  So, in order to
     get this example to run, you must comment out the readline() call in
     apps/nshlib/nsh_consolemain.c like::

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
                ret = readline(pstate->cn_line, CONFIG_NSH_LINELEN,
                               INSTREAM(pstate), OUTSTREAM(pstate));
                if (ret > 0)
         @@ -153,6 +155,7 @@
                            "readline", NSH_ERRNO_OF(-ret));
                    nsh_exit(&pstate->cn_vtbl, 1);
                  }
         +#endif // REMOVE ME
              }

            /* Clean up */

     UPDATE:  I recently implemented a good UART simulation to drive the
     serial console.  So I do not believe that problem exists and I think that
     the above workaround should no longer be necessary. However, I will leave
     the above text in place until I get the opportunity to verify that the
     new UART simulation fixes the problem.

  2019-05-04:  Something has changed.  Today this configuration failed to
     build because is requires CONFIG_NX_XYINPUT=y in the configuration. That
     indicates mouse or touchscreen support.  Apparently, the current NxWM
     will not build without this support.

ostest
------

The "standard" NuttX apps/examples/ostest configuration.

pf_ieee802154
-------------

This is the configuration that used for unit level test of the socket
support for the PF_IEEE802154 address family.  It uses the IEEE 802.15.4
loopback network driver and the test at apps/examples/pf_ieee802154.

Basic usage example::

    nsh> pfserver ab:cd &
    nsh> pfclient ab:cd

pktradio
--------

This configuration is identical to the 'sixlowpan configuration described
below EXCEPT that it uses the generic packet radio loopback network device.

rpproxy and rpserver
--------------------

  This is an example implementation for OpenAMP based on the share memory.

  rpproxy:  Remote slave(client) proxy process.
            rpproxy created a proxy between client and server to allow
            the client to access the hardware resources on different
            process.

  rpserver: Remote master(host) server process.
            rpserver contains all the real hardware configuration, such as:
              1. Universal Asynchronous Receiver/Transmitter (UART).
              2. Specific File System.
              3. Network protocol stack and real network card device.
              4. ...

Rpmsg driver used in this example include:

1. Rpmsg Syslog

    Source::

      include/nuttx/syslog/syslog_rpmsg.h
      drivers/syslog/syslog_rpmsg_server.c
      drivers/syslog/syslog_rpmsg.c

    Describe::

      1>Redirect log to master core
        Linux kernel, NuttX, Freertos ...
      2>Work as early as possible
        Two phase initialization
      3>Never lost the log
        Hang during boot or runtime
        Full system crash(panic, watchdog ...)

2. Rpmsg TTY(UART)

    Source::

      include/nuttx/serial/uart_rpmsg.h
      drivers/serial/uart_rpmsg.c

    Describe::

      1>Like pseudo terminal but between two CPU
      2>No different from real tty(open/read/write/close)
      3>Full duplex communication
      4>Support multiple channels as need
        1)Connect RTOS shell
        2)Make integrated GPS like external(NMEA)
        3)Make integrated modem like external(ATCMD)

3. RpmsgFS

    Source::

      fs/rpmsgfs/rpmsgfs.h
      fs/rpmsgfs/rpmsgfs.c
      fs/rpmsgfs/rpmsgfs_client.c
      fs/rpmsgfs/rpmsgfs_server.c

    Describe::

      1.Like NFS but between two CPU
      2.Fully access remote(Linux/NuttX) File system
        1)Save the tuning parameter during manufacture
        2)Load the tuning parameter file in production
        3)Save audio dump to file for tuning/debugging
        4)Dynamic loading module from remote

4. Rpmsg Net

    Source::

      include/nuttx/net/rpmsg.h
      include/nuttx/net/rpmsgdrv.h
      drivers/net/rpmsgdrv.c
      drivers/usrsock/usrsock_rpmsg.h
      drivers/usrsock/usrsock_rpmsg.c
      drivers/usrsock/usrsock_rpmsg_server.c

    Describe::

      1)Rpmsg UsrSock client
      2)Rpmsg UsrSock server
      3)Rpmsg Net driver
      4)Rpmsg MAC/PHY adapter

To use this example:

1. Build images

    1. Build rpserver and backup the image::

          ./tools/configure.sh sim:rpserver
          make
          cp nuttx ~/rpserver

    2. Distclean the build environment::

          make distclean

    3. Build rpproxy::

          ./tools/configure.sh sim:rpproxy
          make
          cp nuttx ~/rpproxy

2. Test the Rpmsg driver

    1. Rpmsg Syslog:

      Start rpserver::

          $ sudo ~/rpserver
          [    0.000000] server: SIM: Initializing

          NuttShell (NSH)
          server>

          Start rpproxy:

          $ sudo ~/rpproxy

          Check the syslog from rpproxy in rpserver terminal:

          server> [    0.000000] proxy: SIM: Initializing

    2. Rpmsg TTY(UART):

      Use cu switch the current CONSOLE to the proxy::

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

   Mount the remote file system via RPMSGFS, cu to proxy first::

      server> cu
      proxy> mount -t rpmsgfs -o cpu=server,fs=/proc proc_server
      proxy> ls
      /:
        dev/
        etc/
        proc/
        proc_server/
        tmp/

   Check the uptime::

      proxy> cat proc/uptime
        833.21
      proxy> cat proc_server/uptime
        821.72

4. Rpmsg UsrSock:

   "rptun proxy" kernel thread is running::

      server> ps
        PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK COMMAND
          0     0   0 FIFO     Kthread N-- Ready              00000000 000000 Idle Task
          1     1 224 FIFO     Kthread --- Waiting  Signal    00000000 002032 hpwork
          2     1 100 FIFO     Task    --- Running            00000000 004080 init
          3     3 224 FIFO     Kthread --- Waiting  Signal    00000002 002000 rptun proxy 0x56634fa0

      send ICMP ping to network server via rpmsg usrsock:

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
stack.  It enables networking with 6LoWPAN support and uses only a
IEEE802.15.4 MAC loopback network device to supported testing.

This configuration includes apps/examples/nettest and
apps/examples/udpblaster. Neither are truly functional.  The only intent of
this configuration is to verify that the 6LoWPAN stack correctly encodes
IEEE802.15.4 packets on output to the loopback device and correctly decodes
the returned packet.

See also the 'pktradio' configuration.

rtptools
--------

**RTP Tools** is a set of small applications that can be used for processing RTP data.

-  ``rtpplay``: playback RTP sessions recorded by ``rtpdump``
-  ``rtpsend``: generate RTP packets from the textual description, generated by hand or ``rtpdump``
-  ``rtpdump``: parse and print RTP packets, generating output files suitable for ``rtpplay`` and ``rtpsend``
-  ``rtptrans``: RTP translator between unicast and multicast networks

This configuration is based on the :ref:`sim:tcpblaster <simulator_accessing_the_network>` and
builds the ``rtpdump``. This application is able to receive RTP packets and print the contents.
As a real-world application, one could write the received content to a FIFO and play it with
``nxplayer``.

To build it, follow the instructions for :ref:`Accessing the Network <simulator_accessing_the_network>`.

.. tip::
  One can use ``pulseaudio`` to send RTP packets through the network::

    pactl load-module module-null-sink sink_name=rtp format=s16le channels=2 rate=44100 sink_properties="device.description='RTP'"
    pactl load-module module-rtp-send source=rtp.monitor format=s16le destination_ip=10.0.1.2 port=46998

  The loaded sink ``RTP`` is used to send PC's audio to the ``10.0.1.2:46998`` address (SIM's IP).

After being able to access the network through the simulator, run::

  nsh> rtpdump -F short /46998 &
  rtpdump [5:100]
  nsh> 42949704.930000 1277462397 15308
  42949704.930000 1277462714 15309

For a real-world application, check :ref:`RTP Tools on ESP32-LyraT board <esp32-lyrat_rtptools>`.

spiffs
------

This is a test of the SPIFFS file system using the apps/testing/fstest test
with an MTD RAM driver to simulate the FLASH part.

sotest
------

This is a configuration to test CONFIG_LIBC_MODLIB with 64-bit modules.
This has apps/examples/sotest enabled.
This configuration is intended for 64-bit host OS.

sotest32
--------

This is a configuration to test CONFIG_LIBC_MODLIB with CONFIG_SIM_M32
and 32-bit modules.
This has apps/examples/sotest enabled.
This configuration is intended for 64-bit host OS.

tcploop
-------

This configuration performs a TCP "performance" test using
apps/examples/tcpblaster and the IPv6 local loopback device.  Performance
is in quotes because, while that is the intent of the tcpblaster example,
this is not an appropriate configuration for TCP performance testing.
Rather, this configurat is useful only for verifying TCP transfers over
the loopback device.

To use IPv4, modify these settings in the defconfig file::

    -# CONFIG_NET_IPv4 is not set
    -CONFIG_NET_IPv6=y
    -CONFIG_NET_IPv6_NCONF_ENTRIES=4

touchscreen
-----------

This configuration uses the simple touchscreen test at
apps/examples/touchscreen.  This test will create an empty X11 window and
will print the touchscreen output as it is received from the simulated
touchscreen driver.

Since this example uses the simulated frame buffer driver, most of the
configuration settings discussed for the "nx11" configuration also apply
here.  See that discussion above.

See apps/examples/README.txt for further information about build
requirements and configuration settings.

toywasm
-------

This is a configuration with toywasm.

An example usage::

    NuttShell (NSH) NuttX-10.4.0
    nsh> mount -t hostfs -o fs=/tmp/wasm /mnt
    nsh> toywasm --wasi /mnt/hello.wasm
    hello
    nsh>

udgram
------

This is the same as the nsh configuration except that it includes two
additional built in applications:  server and client.  These applications
are provided by the test at apps/examples/udgram. This configuration enables
local, Unix domain sockets and supports the test of the datagram sockets.

To use the test:

    nsh> server &
    nsh> client

unionfs
-------

This is a version of NSH dedicated to performing the simple test of the
Union File System at apps/examples/unionfs.  The command 'unionfs' will mount
the Union File System at /mnt/unionfs.  You can than compare what you see at
/mnt/unionfs with the content of the ROMFS file systems at
apps/examples/unionfs/atestdir and btestdir.

Here is some sample output from the test::

    NuttShell (NSH)
    nsh> unionfs
    Mounting ROMFS file system 1 at target=/mnt/a with source=/dev/ram4
    Mounting ROMFS file system 2 at target=/mnt/b with source=/dev/ram5
    nsh> ls /mnt/unionfs
    /mnt/unionfs:
     .
     afile.txt
     offset/

When unionfs was created, file system was joined with an offset called
"offset".  Therefore, all of the file system 2 root contents will appear to
reside under a directory called offset/ (although there is no directory
called offset/ on file system 2).  File system 1 on the other hand does
have an actual directory called offset/.  If we list the contents of the
offset/ directory in the unified file system, we see the merged contents of
the file system 1 offset/ directory and the file system 2 root directory::

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
    This is a file in the offset/ directory on file system 1

    nsh> cat /mnt/unionfs/offset/bfile.txt
    This is another file in the root directory on file system 2

The directory offset/adir exists on file system 1 and the directory adir/
exists on file system 2.  You can see that these also overlap::

    nsh> ls /mnt/unionfs/offset/adir
    /mnt/unionfs/offset/adir:
     ..
     asubdir/
     adirfile.txt
     bsubdir/
     bdirfile.txt
     .

The unified directory listing is showing files from both file systems in
their respective offset adir/ subdirectories.  The file adirfile.txt exists
in both file system 1 and file system 2 but the version in file system 2 is
occluded by the version in file system 1.  The only way that you can know
which you are looking at is by cat'ing the file::

    nsh> cat /mnt/unionfs/offset/adir/adirfile.txt
    This is a file in directory offset/adir on file system 1

The file on file system 1 has correctly occluded the file with the same name
on file system 2.  bdirfile.txt, however, only exists on file system 2, so
it is not occluded::

    nsh> cat /mnt/unionfs/offset/adir/bdirfile.txt
    This is another file in directory adir on file system 2

You can see the files in the two file systems before they were unified at
apps/examples/unionfs/atestdir and btestdir.

userfs
------

  This is another NSH configuration that includes the built-in application of
  apps/examples/userfs to support test of the UserFS on the simulation
  platform.

  To use the test::

    nsh> userfs                 # Mounts the UserFS test file system at
                                # /mnt/ufstest
    nsh> mount                  # Testing is then performed by exercising the
                                # file system from the command line
    nsh> ls -l /mnt/ufstest
    nsh> cat /mnt/ufstest/File1
    etc.

ustream
-------

  This is the same as the nsh configuration except that it includes two
  addition built in applications:  server and client.  These applications are
  provided by the test at apps/examples/ustream. This configuration enables
  local, Unix domain sockets and supports the test of the stream sockets.

  To use the test::

    nsh> server &
    nsh> client

  Note that the binfs file system is mounted at /bin when the system starts
  up.

vncserver
---------

  This a simple vnc server test configuration, Remmina is tested and recommended since
  there are some compatibility issues. By default SIM will be blocked at startup to
  wait client connection, if a client connected, then the fb example will launch.

vpnkit
------

  This is a configuration with VPNKit support.  See NETWORK-VPNKIT.txt.

wamr
----

This is a configuration for WebAssembly sample.

1. Compile Toolchain

   1. Download WASI sdk and export the WASI_SDK_PATH path

    .. code-block:: console

      wget https://github.com/WebAssembly/wasi-sdk/releases/download/wasi-sdk-19/wasi-sdk-19.0-linux.tar.gz
      tar xf wasi-sdk-19.0-linux.tar.gz
      # Put wasi-sdk-19.0 to your host WASI_SDK_PATH environment variable, like:
      export WASI_SDK_PATH=`pwd`/wasi-sdk-19.0

   2. Download Wamr "wamrc" AOT compiler and export to the PATH

    .. code-block:: console

      mkdir wamrc
      wget https://github.com/bytecodealliance/wasm-micro-runtime/releases/download/WAMR-1.1.2/wamrc-1.1.2-x86_64-ubuntu-20.04.tar.gz
      tar xf wamrc-1.1.2-x86_64-ubuntu-20.04.tar.gz
      export PATH=$PATH:$PWD

2. Configuring and running

   1. Configuring sim/wamr and compile

    .. code-block:: console

          ./tools/configure.sh  sim/wamr
          make
          ...
          Wamrc Generate AoT: /home/archer/code/nuttx/n5/apps/wasm/hello.aot
          Wamrc Generate AoT: /home/archer/code/nuttx/n5/apps/wasm/coremark.aot
          LD:  nuttx

   2. Copy the generated wasm file(Interpreter/AoT)

    .. code-block:: console

      cp ../apps/wasm/hello.aot .
      cp ../apps/wasm/hello.wasm .
      cp ../apps/wasm/coremark.wasm .

   3. Run iwasm

    .. code-block:: console

          ./nuttx
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

  Get Raw Gadget:
  Get Raw Gadget code at https://github.com/xairy/raw-gadget.

  Make Raw Gadget:
  Run make in the raw_gadget and dummy_hcd directory. If raw_gadget build
  fail, you need to check which register interface meets your kernel version,
  usb_gadget_probe_driver or usb_gadget_register_driver.

  Install Raw Gadget:
  Run ./insmod.sh in the raw_gadget and dummy_hcd directory.

2. Configuration

  sim:usbdev contains two different sets of composite devices::

    conn0: adb & rndis
    conn1: cdcacm & cdcecm

  You can use the sim:usbdev configuration::

    ./tools/configure.sh sim:usbdev

3. How to run

  Run nuttx with root mode, then you can use it as the following::

    1> Run ADB:

  NuttX enter command::

      $ conn 0
      $ adbd &

  Host PC enter the ADB command::

      $ adb kill-server
      $ adb devices
      List of devices attached
      * daemon not running; starting now at tcp:5037
      * daemon started successfully
      0101        device

  If ADB connection fails, make sure the udev rule is added correctly.
  Edit /etc/udev/rules.d/51-android.rules file and add the following to it:
  SUBSYSTEM=="usb", ATTR{idVendor}=="1630", ATTR{idProduct}=="0042", MODE="0666", GROUP="plugdev"

  Then you can use commands such as adb shell, adb push, adb pull as normal.

    2> Run RNDIS:
  
  NuttX enter command::

      $ conn 0
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:00:00:00:00:00 at UP
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ dhcpd_start eth0
      eth0    Link encap:Ethernet HWaddr 00:00:00:00:00:00 at UP
            inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

  Host PC, you can see the network device named usb0::

      $ ifconfig
      usb0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 602
              inet 10.0.0.4  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 36:50:3d:62:b5:80  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 43  bytes 8544 (8.5 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

  Then you can test the network connection using the ping command or telnet.

    3> Run CDCACM:

  NuttX enter command::

      $ conn 1

  If the connection is successful, you can see /dev/ttyACM devices on both NuttX
  and host PC.

  Then you can use echo and cat command to test:

  NuttX::

      nsh> echo hello > /dev/ttyACM0

  Host PC::

      $ cat /dev/ttyACM0
      hello

    3> Run CDCECM:

  NuttX enter command::

      $ conn 1
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
              inet addr:0.0.0.0 DRaddr:0.0.0.0 Mask:0.0.0.0
      $ dhcpd_start eth0
      $ ifconfig
      eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
              inet addr:10.0.0.1 DRaddr:10.0.0.1 Mask:255.255.255.0

  Host PC, you can see the network device named enx020000112233::

      $ ifconfig
      enx020000112233: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 576
              inet 10.0.0.4  netmask 255.255.255.0  broadcast 10.0.0.255
              ether 02:00:00:11:22:33  txqueuelen 1000  (以太网)
              RX packets 0  bytes 0 (0.0 B)
              RX errors 0  dropped 0  overruns 0  frame 0
              TX packets 58  bytes 9143 (9.1 KB)
              TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

  Then you can test the network connection using the ping command or telnet.

usbhost
-------

This is a configuration with sim usbhost support.

1. Libusb1.0 setup::

    $ sudo apt-get -y install libusb-1.0-0-dev
    $ sudo apt-get -y install libusb-1.0-0-dev:i386

2. Configuration

   sim:usbhost support cdcacm.

   You can use the sim:usbdev configuration::

    $ ./tools/configure.sh sim:usbhost

   Configure the device you want to connet::

    CONFIG_SIM_USB_PID=0x0042
    CONFIG_SIM_USB_VID=0x1630

3. How to run

   Run sim usbhost with root mode, run sim usbdev or plug-in cdcacm usb device.
   Then you can use /dev/ttyACM to transfer data.


README.txt
==========

.. include:: README.txt
   :literal:
