README
^^^^^^

Contents
^^^^^^^^
  o Overview
    - Description
    - Fake Interrupts
    - Timing Fidelity
  o Debugging
  o Issues
    - 64-bit Issues
    - Compiler differences
    - Stack Size Issues
    - Symbol Collisions
    - Networking Issues
    - X11 Issues
    - Cygwin64 Issues
    - SMP
  o BASIC
  o Configurations

Overview
^^^^^^^^

Description
-----------
This README file describes the contents of the build configurations
available for the NuttX "sim" target.  The sim target is a NuttX port that
runs as a user-space program under Linux or Cygwin.  It is a very "low
fidelity" embedded system simulation:  This environment does not support
any kind of asynchronous events -- there are nothing like interrupts in this
context.  Therefore, there can be no pre-empting events.

Fake Interrupts
---------------
In order to get timed behavior, the system timer "interrupt handler" is
called from the sim target's IDLE loop.  The IDLE runs whenever there is no
other task running.  So, for example, if a task calls sleep(), then that
task will suspend wanting for the time to elapse.  If nothing else is
available to run, then the IDLE loop runs and the timer increments,
eventually re-awakening the sleeping task.

Context switching is based on logic similar to setjmp() and longjmp().

The sim target is used primarily as a development and test platform for new
RTOS features.  It is also of academic interest.  But it has no real-world
application that I know of.

Timing Fidelity
---------------
NOTE:  In order to facility fast testing, the sim target's IDLE loop, by
default, calls the system "interrupt handler" as fast as possible.  As a
result, there really are no noticeable delays when a task sleeps.  However,
the task really does sleep -- but the time scale is wrong.  If you want
behavior that is closer to normal timing, then you can define
CONFIG_SIM_WALLTIME=y in your configuration file.  This configuration
setting will cause the sim target's IDLE loop to delay on each call so that
the system "timer interrupt" is called at a rate approximately correct for

the system timer tick rate.  With this definition in the configuration,
sleep() behavior is more or less normal.

Debugging
^^^^^^^^^
One of the best reasons to use the simulation is that is supports great, Linux-
based debugging.  Here are the steps that I following to use the Linux ddd
graphical front-end to GDB:

1. Modify the top-level configuration file.  Enable debug symbols by defining
   the following.

   cd <NuttX-Directory>
   CONFIG_DEBUG_SYMBOLS=y

2. Re-build:

   cd <NuttX-Directory>
   make clean
   make

3. Then start the debugging:

   ddd nuttx &
   gdb> b user_start
   gdb> r

NOTE:  This above steps work fine on both Linux and Cygwin.  On Cygwin, you
will need to start the Cywin-X server before running ddd.

Issues
^^^^^^

64-Bit Issues
-------------
As mentioned above, context switching is based on logic like setjmp() and
longjmp().  This context switching is available for 32-bit and 64-bit
targets.  You must, however, set the correct target in the configuration
before you build: CONFIG_HOST_X86_64 or CONFIG_HOST_X86 for 64- and 32-bit
targets, respectively.  On a 64-bit machine, you can also force the 32-bit
build with CONFIG_SIM_M32=y (which does not seem to be supported by more
contemporary x86_64 compilers).

There are other 64-bit issues as well.  For example, addresses are retained in
32-bit unsigned integer types in a few places.  On a 64-bit machine, the 32-bit
address storage may corrupt 64-bit addressing.  NOTE:  This is really a bug --
addresses should not be retained in uint32_t types but rather in uintptr_t types
to avoid issues just like this.

Compiler differences
--------------------

operator new:

  Problem:     "'operator new' takes size_t ('...') as first parameter"
  Workaround:   Add -fpermissive to the compilation flags

Stack Size Issues
-----------------
When you run the NuttX simulation, it uses stacks allocated by NuttX from the
NuttX heap.  The memory management model is exactly the same in the simulation
as it is real, target system.  This is good because this produces a higher
fidelity simulation.

However, when the simulation calls into Linux/Cygwin libraries, it will still
use these small simulation stacks.  This happens, for example, when you call
into the system to get and put characters to the console window or when you
make x11 calls into the system.  The programming model within those libraries
will assume a Linux/Cygwin environment where the stack size grows dynamically
and not the small, limited stacks of a deeply embedded system.

As a consequence, those system libraries may allocate large data structures
on the stack and overflow the small NuttX stacks.  X11, in particular,
requires large stacks.  If you are using X11 in the simulation, make sure
that you set aside a "lot" of stack for the X11 system calls (maybe 8 or 16Kb).
The stack size for the thread that begins with user start is controlled
by the configuration setting CONFIG_USERMAIN_STACKSIZE; you may need to
increase this value to larger number to survive the X11 system calls.

If you are running X11 applications as NSH add-on programs, then the stack
size of the add-on program is controlled in another way.  Here are the
steps for increasing the stack size in that case:

  cd ../apps/builtin    # Go to the builtin apps directory
  vi builtin_list.h     # Edit this file and increase the stack size of the add-on
  rm .built *.o         # This will force the builtin apps logic to rebuild

Symbol Collisions
-----------------
The simulation build is a two pass build:

  1. On the first pass, an intermediate, partially relocatable object is
    created called nuttx.rel.  This includes all of the files that are part
    of the NuttX "domain."

  2. On the second pass, the files are are in the host OS domain are build
     and then linked with nuttx.rel to generate the simulation program.

NuttX is a POSIX compliant RTOS and is normally build on a POSIX compliant
host environment (like Linux or Cygwin).  As a result, the same symbols are
exported by both the NuttX doman and the host domain.  How can we keep them
separate?

This is done using the special file nuttx-name.dat.  This file just contains
a list of original function names and a new function name.  For example
the NuttX printf() will get the new name NXprintf().

This nuttx-names.dat file is used by the objcopy program between pass1 and
pass2 to rename all of the symbols in the nuttx.rel object so that they do
not collide with names provided by the host OS in the host PC domain.

Occasionally, as you test new functionality, you will find that you need to
add more names to the nuttx-names.dat file.  If there is a missing name
mapping in nuttx-name.dat, the symptoms may be very obscure and difficult to
debug.  What happens in this case is that when logic in nuttx.rel intended
to call the NuttX domain function, it instead calls into the host OS
function of the same name.

Often you can survive such events.  For example, it really should not matter
which version of strlen() you call.  Other times, it can cause subtle,
mysterious errors.  Usually, however, callng the wrong function in the wrong
OS results in a fatal crash.

Networking Issues
-----------------
I never did get networking to work on the sim target.  It tries to use the
tap device (/dev/net/tun) to emulate an Ethernet NIC, but I never got it
correctly integrated with the NuttX networking (I probably should try using
raw sockets instead).

Update:  Max Holtzberg reports to me that the tap device actually does work
properly, but not in an NSH configuration because of stdio operations freeze
the simulation.

REVISIT: This may not long be an issue even with NSH because of the recent
redesign of how the stdio devices are handled in the simulation (they should
no longer freeze the simulation).

X11 Issues
----------
There is an X11-based framebuffer driver that you can use exercise the NuttX
graphics subsystem on the simulator (see the sim/nx11 configuration below).
This may require a lot of tinkering to get working, depending upon where
your X11 installation stores libraries and header files and how it names
libraries.

For example, on Ubuntu 9.09, I had to do the following to get a clean build:

    cd /usr/lib/
    sudo ln -s libXext.so.6.4.0 libXext.so

(I also get a segmentation fault at the conclusion of the NX test -- that
will need to get looked into as well).

The X11 examples builds on Cygwin, but does not run.  The last time I tried
it, XOpenDisplay() aborted the program.  UPDATE:  This was caused by the
small stack size and can be fixed by increasing the size of the NuttX stack
that calls into X11.  See the discussion "Stack Size Issues" above.

Cygwin64 Issues
---------------
There are some additional issues using the simulator with Cygwin64.  Below
is the summary of the changes that I had to make to get the simulator
working in that environment:

  CONFIG_HOST_X86_64=y
  CONFIG_SIM_M32=n
    Need to select X64_64.  Cygwin64 tools do not seem to support any option
    to build a 32-bit target.

  CONFIG_SIM_CYGWIN_DECORATED=n
    Older versions of Cygwin toolsdecorated C symbol names by adding an
    underscore to the beginning of the symbol name.  Newer versions of
    Cygwin do not seem to do this.  Deselecting CONFIG_SIM_CYGWIN_DECORATED
    will select the symbols without the leading underscore as needed by
    the Cygwin64 toolchain.

    How do you know if you need this option?  You could look at the generated
    symbol tables to see if there are underscore characters at the beginning
    of the symbol names.  Or, if you need this option, the simulation will not
    run:  It will crash early, probably in some function due to the failure to
    allocate memory.

    In this case, when I tried to run nutt.exe from the command line, it
    exited silently.  Running with GDB I get following (before hitting a
    breakpoint at main()):

      (gdb) r
      Starting program: /cygdrive/c/Users/Gregory/Documents/projects/nuttx/master/nuttx/nuttx.exe
      [New Thread 6512.0xda8]
      [New Thread 6512.0x998]
            1 [main] nuttx 6512 C:\Users\Gregory\Documents\projects\nuttx\master\nuttx\nuttx.exe: *** fatal error - Internal error: Out of memory for new path buf.
          736 [main] nuttx 6512 cygwin_exception::open_stackdumpfile: Dumping stack trace to nuttx.exe.stackdump
      [Thread 6512.0x998 exited with code 256]
      [Inferior 1 (process 6512) exited with code 0400]

  CONFIG_SIM_X8664_SYSTEMV=n
  CONFIG_SIM_X8664_MICROSOFT=y
    Selet Microsoft x64 calling convention.

    The Microsoft x64 calling convention is followed on Microsoft Windows and
    pre-boot UEFI (for long mode on x86-64). It uses registers RCX, RDX, R8,
    R9 for the first four integer or pointer arguments (in that order), and
    XMM0, XMM1, XMM2, XMM3 are used for floating point arguments. Additional
    arguments are pushed onto the stack (right to left). Integer return
    values (similar to x86) are returned in RAX if 64 bits or less. Floating
    point return values are returned in XMM0. Parameters less than 64 bits
    long are not zero extended; the high bits are not zeroed.

SMP
---
  The configuration has basic support SMP testing.  The simulation supports
  the emulation of multiple CPUs by creating multiple pthreads, each run a
  copy of the simulation in the same process address space.

  At present, the SMP simulation is not fully functional:  It does operate
  on the simulated CPU threads for a few context switches then fails during
  a setjmp() operation.  I suspect that this is not an issue with the NuttX
  SMP logic but more likely some chaos in the pthread controls. I have seen
  similar such strange behavior other times that I have tried to use
  setjmp/longmp from a signal handler! Like when I tried to implement
  simulated interrupts using signals.

  Apparently, if longjmp is invoked from the context of a signal handler,
  the result is undefined: http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1318.htm

  You can enable SMP for ostest configuration by enabling:

    +CONFIG_SPINLOCK=y
    +CONFIG_SMP=y
    +CONFIG_SMP_NCPUS=2
    +CONFIG_SMP_IDLETHREAD_STACKSIZE=2048

  You also must enable near-realtime-performance otherwise even long
  timeouts will expire before a CPU thread even has a chance to execute.

    -# CONFIG_SIM_WALLTIME is not set
    +CONFIG_SIM_WALLTIME=y

  And you can enable some additional debug output with:

    -# CONFIG_DEBUG_SCHED is not set
    +CONFIG_DEBUG_SCHED=y

    -# CONFIG_SCHED_INSTRUMENTATION is not set
    +CONFIG_SCHED_INSTRUMENTATION=y

  The SMP configuration will run with:

    CONFIG_SMP_NCPUS=1

  In this case there is, of course, no muli-CPU processing, but this does
  verify the correctness of some the basic SMP logic.

  The NSH configuration can also be forced to run SMP, but suffers from
  the same quirky behavior.  I can be made reliable if you modify
  arch/sim/src/up_idle.c so that the IDLE loop only runs for CPU0.
  Otherwise, often simuart_post() will be called from CPU1 and it will
  try to restart NSH on CPU0 and, again, the same quirkiness occurs.

  But for example, this command:

    nsh> sleep 1 &

  will execute the sleep command on CPU1 which has worked every time
  that I have tried it (which is not too many times).

BASIC
^^^^^

  I have used the sim/nsh configuration to test Michael Haardt's BASIC interpreter
  that you can find at apps/interpreters/bas.

    Bas is an interpreter for the classic dialect of the programming language
    BASIC.  It is pretty compatible to typical BASIC interpreters of the 1980s,
    unlike some other UNIX BASIC interpreters, that implement a different
    syntax, breaking compatibility to existing programs.  Bas offers many ANSI
    BASIC statements for structured programming, such as procedures, local
    variables and various loop types.  Further there are matrix operations,
    automatic LIST indentation and many statements and functions found in
    specific classic dialects.  Line numbers are not required.

  There is also a test suite for the interpreter that can be found at
  apps/examples/bastest.

  Configuration
  -------------
  Below are the recommended configuration changes to use BAS with the
  stm32f4discovery/nsh configuration:

  Dependencies:
    CONFIG_LIBC_EXECFUNCS=y      : exec*() functions are required
    CONFIG_LIBM=y                : Some floating point library is required
    CONFIG_LIBC_FLOATINGPOINT=y  : Floating point printing support is required
    CONFIG_LIBC_TMPDIR="/tmp"    : Writeable temporary files needed for some commands

  Enable the BASIC interpreter.  Other default options should be okay:
    CONFIG_INTERPRETERS_BAS=y    : Enables the interpreter
    CONFIG_INTERPRETER_BAS_VT100=y

  The BASIC test suite can be included:
     CONFIG_FS_ROMFS=y           : ROMFS support is needed
     CONFIG_EXAMPLES_BASTEST=y   : Enables the BASIC test setup
     CONFIG_EXAMPLES_BASTEST_DEVMINOR=6
     CONFIG_EXAMPLES_BASTEST_DEVPATH="/dev/ram6"

  Usage
  -----
  This setup will initialize the BASIC test (optional):  This will mount
  a ROMFS file system at /mnt/romfs that contains the BASIC test files:

  nsh> bastest
  Registering romdisk at /dev/ram6
  Mounting ROMFS filesystem at target=/mnt/romfs with source=/dev/ram6
  nsh>

  The interactive interpreter is started like:

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

  Or you can load a test into memory and execute it interactively:

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
^^^^^^^^^^^^^^

Common Configuration Information
--------------------------------

  1. Each configuration is maintained in a sub-directory and can be selected
     as follow:

       tools/configure.sh sim/<subdir>

     Where <subdir> is one of the following sub-directories.

  2. All configurations uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the kconfig mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. Before building, make sure that the configuration is correct for you host platform:

     a. Linux, 32-bit CPU

        CONFIG_HOST_LINUX=y
        CONFIG_HOST_WINDOWS=n
        CONFIG_HOST_X86=y
        CONFIG_HOST_X86_64=n

     b. Linux, 64-bit CPU, 32-bit build

        CONFIG_HOST_LINUX=y
        CONFIG_HOST_WINDOWS=n
        CONFIG_HOST_X86=n
        CONFIG_HOST_X86_64=y
        CONFIG_SIM_X8664_MICROSOFT=n
        CONFIG_SIM_X8664_SYSTEMV=y
        CONFIG_SIM_M32=y

     c. Linux, 64-bit CPU, 64-bit build

        CONFIG_HOST_LINUX=y
        CONFIG_HOST_WINDOWS=n
        CONFIG_HOST_X86=n
        CONFIG_HOST_X86_64=y
        CONFIG_SIM_X8664_MICROSOFT=n
        CONFIG_SIM_X8664_SYSTEMV=y
        CONFIG_SIM_M32=n

     d. Cygwin, 32-bit

        CONFIG_HOST_LINUX=n
        CONFIG_HOST_WINDOWS=y
        CONFIG_WINDOWS_CYGWIN=y
        CONFIG_HOST_X86=y
        CONFIG_HOST_X86_64=n

     e. Cygwin64, 64-bit, 32-bit build

        I don't believe this configuration is supported by Cygwin64

     f. Cygwin64, 64-bit, 64-bit build

        CONFIG_HOST_LINUX=n
        CONFIG_HOST_WINDOWS=y
        CONFIG_WINDOWS_CYGWIN=y
        CONFIG_HOST_X86=n
        CONFIG_HOST_X86_64=y
        CONFIG_SIM_X8664_MICROSOFT=y
        CONFIG_SIM_X8664_SYSTEMV=n
        CONFIG_SIM_M32=n

Configuration Sub-Directories
-----------------------------

bluetooth

  Supports some very limited, primitive, low-level debug of the Bluetoot
  stack using the Bluetooth "Swiss Army Knife" at apps/wireless/bluetooth/btsak
  and the NULL Bluetooth device at drivers/wireless/bluetooth/bt_null.c

configdata

  A unit test for the MTD configuration data driver.

cxxtest


  The C++ standard libary test at apps/examples/cxxtest configuration.  This
  test is used to verify the uClibc++ port to NuttX.

  NOTES
  -----
  1. Before you can use this example, you must first install the uClibc++
     C++ library.  This is located outside of the NuttX source tree in the
     NuttX uClibc++ GIT repository.  See the README.txt file there for
     instructions on how to install uClibc++

  2. At present (2012/11/02), exceptions are disabled in this example
     CONFIG_UCLIBCXX_EXCEPTION=n).  It is probably not necessary to
     disable exceptions.

  3. Unfortunately, this example will not run now.

     The reason that the example will not run on the simulator has
     to do with when static constructors are enabled:  In the simulator
     it will attempt to execute the static constructors before main()
     starts. BUT... NuttX is not initialized and this results in a crash.

     To really use this example, I will have to think of some way to
     postpone running C++ static initializers until NuttX has been
     initialized.

fb

  A simple configuration used for some basic (non-graphic) debug of the
  framebuffer character drivers using apps/examples/fb.

ipforward

  This is an NSH configuration that includes a simple test of the NuttX
  IP forwarding logic using apps/examples/ipforward.  That example uses
  two TUN network devices to represent two networks.  The test then sends
  packets from one network destined for the other network.  The NuttX IP
  forwarding logic will recognize that the received packets are not destined
  for it and will forward the logic to the other TUN network.  The
  application logic then both sends the packets on one network and receives
  and verifies the forwarded packet recieved on the other network.  The
  received packets differ from the sent packets only in that the hop limit
  (TTL) has been decremented.

  Be default, this test will forward TCP packets.  The test can be modified
  to support forwarding of ICMPv6 multicast packets with these changes to
  the .config file:

    -CONFIG_EXAMPLES_IPFORWARD_TCP=y
    +CONFIG_EXAMPLES_IPFORWARD_ICMPv6=y

    +CONFIG_NET_ICMPv6=y
    +CONFIG_NET_ICMPv6_SOCKET=y
    +CONFIG_NET_ETHERNET=y
    +CONFIG_NET_IPFORWARD_BROADCAST=y

  Additional required settings will also be selected when you manually
  select the above via 'make menuconfig'.

loadable

  This configuration provides an example of loadable apps.  It cannot used
  with any Windows configuration, however, because Windows does not use
  the ELF format.

minibasic

  This configuration was used to test the Mini Basic port at
  apps/interpreters/minibasic.

mount

  Configures to use apps/examples/mount.

mtdpart

  This is the apps/examples/mtdpart test using a MTD RAM driver to
  simulate the FLASH part.

mtdrwb

  This is the apps/examples/mtdrwb test using a MTD RAM driver to
  simulate the FLASH part.

nettest

  Configures to use apps/examples/nettest.  This configuration
  enables networking using the network TAP device.

  NOTES:

  1. The NuttX network is not, however, functional on the Linux TAP
     device yet.

     UPDATE:  The TAP device does apparently work according to a NuttX
     user (provided that it is not used with NSH: NSH waits on readline()
     for console input.  When it calls readline(), the whole system blocks
     waiting from input from the host OS).  My failure to get the TAP
     device working appears to have been a cockpit error.

  2. As of NuttX-5.18, when built on Windows, this test does not try
     to use the TAP device (which is not available on Cygwin anyway),
     but inside will try to use the Cygwin WPCAP library.  Only the
     most preliminary testing has been performed with the Cygwin WPCAP
     library, however.

     NOTE that the IP address is hard-coded in arch/sim/src/up_wpcap.c.
     You will either need to edit your configuration files to use 10.0.0.1
     on the "target" (CONFIG_EXAMPLES_NETTEST_*) or edit up_wpcap.c to
     select the IP address that you want to use.

nsh

  Configures to use the NuttShell at apps/examples/nsh.

  NOTES:

  1. This version has one builtin function:  This configuration:
     apps/examples/hello.

  2. This version has password protection enable.  Here is the login:

       USERNAME:  admin
       PASSWORD:  Administrator

     The encrypted password is retained in /etc/passwd.  I am sure that
     you will find this annoying.  You can disable the password protection
     by de-selecting CONFIG_NSH_CONSOLE_LOGIN=y.

  3. This configuration has BINFS enabled so that the builtin applications
     can be made visible in the file system.  Because of that, the
     build in applications do not work as other examples.

     For example trying to execute the hello builtin application will
     fail:

       nsh> hello
       nsh: hello: command not found
       nsh>

     Unless you first mount the BINFS file system:

       nsh> mount -t binfs /bin
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

  This is another example that configures to use the NuttShell at apps/examples/nsh.
  Like nsh, this version uses NSH built-in functions:  The nx, nxhello, and
  nxlines examples are included as built-in functions.

  NOTES:

  1. X11 Configuration

     This configuration uses an X11-based framebuffer driver.  Of course, this
     configuration can only be used in environments that support X11!  (And it
     may not even be usable in all of those environments without some "tweaking"
     See discussion below under the nx11 configuration).

nx

  Configures to use apps/examples/nx.

  NOTES:

  1. Special Framebuffer Configuration

     Special simulated framebuffer configuration options:

       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. No Display!

     This version has NO DISPLAY and is only useful for debugging NX
     internals in environments where X11 is not supported.  There is
     and additional configuration that may be added to include an X11-
     based simulated framebuffer driver:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer

     See the "nx11" configuration below for more information.

nx11

  Configures to use apps/examples/nx.  This configuration is similar
  to the nx configuration except that it adds support for an X11-
  based framebuffer driver.  Of course, this configuration can only
  be used in environments that support X11!  (And it may not even
  be usable in all of those environments without some "tweaking").

  1. Special Framebuffer Configuration

     This configuration uses the same special simulated framebuffer
     configuration options as the nx configuration:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer
       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. X11 Configuration

     But now, since CONFIG_SIM_X11FB is also selected the following
     definitions are needed

       CONFIG_SIM_FBBPP (must match the resolution of the display).
       CONFIG_FB_CMAP=y

     My system has 24-bit color, but packed into 32-bit words so
     the correct setting of CONFIG_SIM_FBBPP is 32.

     For whatever value of CONFIG_SIM_FBBPP is selected, the
     corresponding CONFIG_NX_DISABLE_*BPP setting must not be
     disabled.

  3. Touchscreen Support

     A X11 mouse-based touchscreen simulation can also be enabled
     by setting:

       CONFIG_INPUT=y
       CONFIG_SIM_TOUCHSCREEN=y

     NOTES:

     a. If you do not have the call to sim_tcinitialize(0), the build
        will mysteriously fail claiming that is can't find up_tcenter()
        and up_tcleave().  That is a consequence of the crazy way that
        the simulation is built and can only be eliminated by calling
        up_simtouchscreen(0) from your application.

     b. You must first call up_fbinitialize(0) before calling
        up_simtouchscreen() or you will get a crash.

     c. Call sim_tcunininitializee() when you are finished with the
        simulated touchscreen.

     d. Enable CONFIG_DEBUG_INPUT=y for touchscreen debug output.

  4. X11 Build Issues

     To get the system to compile under various X11 installations
     you may have to modify a few things.  For example, in order
     to find libXext, I had to make the following change under
     Ubuntu 9.09:

       cd /usr/lib/
       sudo ln -s libXext.so.6.4.0 libXext.so

   5. apps/examples/nxterm

      This configuration is also set up to use the apps/examples/nxterm
      test instead of apps/examples/nx.  To enable this configuration,
      First, select Multi-User mode as described above.  Then add the
      following definitions to the defconfig file:

       -CONFIG_NXTERM=n
       +CONFIG_NXTERM=y

       -CONFIG_EXAMPLES_NX=y
       +CONFIG_EXAMPLES_NX=n

       -CONFIG_EXAMPLES_NXTERM=n
       +CONFIG_EXAMPLES_NXTERM=y

     See apps/examples/README.txt for further details.

nxffs

  This is the apps/examples/nxffs test using a MTD RAM driver to
  simulate the FLASH part.

nxlines

  This is the apps/examples/nxlines test.

nxwm

  This is a special configuration setup for the NxWM window manager
  UnitTest.  The NxWM window manager can be found here:

    nuttx-code/NxWidgets/nxwm

  The NxWM unit test can be found at:

    nuttx-code/NxWidgets/UnitTests/nxwm

  Documentation for installing the NxWM unit test can be found here:

    nuttx-code/NxWidgets/UnitTests/READEM.txt

  NOTES

  1. There is an issue with running this example under the
     simulation.  In the default configuration, this example will
     run the NxTerm example which waits on readline() for console
     input.  When it calls readline(), the whole system blocks
     waiting from input from the host OS.  So, in order to get
     this example to run, you must comment out the readline call in
     apps/nshlib/nsh_consolemain.c like:

     Index: nsh_consolemain.c
     ===================================================================
     --- nsh_consolemain.c   (revision 4681)
     +++ nsh_consolemain.c   (working copy)
     @@ -117,7 +117,8 @@
        /* Execute the startup script */

      #ifdef CONFIG_NSH_ROMFSETC
     -  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
     +// REMOVE ME
     +//  (void)nsh_script(&pstate->cn_vtbl, "init", NSH_INITPATH);
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

     UPDATE:  I recently implemented a good UART simulation to driver
     the serial console.  So I do not believe that problem exists and
     I think that the above workaround should no longer be necessary.
     However, I will leave the above text in place until I get then
     oppotunity to verify that the new UART simulation fixes the problem.

ostest

  The "standard" NuttX apps/examples/ostest configuration.

pashello

  Configures to use apps/examples/pashello.

pf_ieee802154

  This is the configuration that used for unit level test of the
  socket support for the PF_IEEE802154 address family.  It uses
  the IEEE 802.15.4 loopback network driver and the test at
  apps/examples/pf_ieee802154.

  Basic usage example:

    nsh> pfserver ab:cd &
    nsh> pfclient ab:cd

pktradio

  This configuration is identical to the 'sixlowpan configuration
  described below EXCEPT that is uses the generic packet radio
  loopback network device.

sixlowpan

  This configuration was intended only for unit-level testing of the
  6LoWPAN stack.  It enables networking with 6LoWPAN support and uses
  only a IEEE802.15.4 MAC loopback network device to supported testing.

  This configuration includes apps/examples/nettest and apps/examples/udpblaster.
  Neither are truly functional.  The only intent of this configuration
  is to verify that the 6LoWPAN stack correctly encodes IEEE802.15.4
  packets on output to the loopback device and correctly decodes the
  returned packet.

  See also the 'pktradio' configuration.

touchscreen

  This configuration uses the simple touchscreen test at
  apps/examples/touchscreen.  This test will create an empty X11 window
  and will print the touchscreen output as it is received from the
  simulated touchscreen driver.

  Since this example uses the simulated frame buffer driver, the
  most of the configuration settings discussed for the "nx11"
  configuration also apply here.  See that discussion above.

  See apps/examples/README.txt for further information about build
  requirements and configuration settings.

traveler

  Configures to build the Traveler first person, 3-D ray casting game at
  apps/graphics/traveler.  This configuration derives fromthe nx11
  configuration and many of the comments there appear here as well.
  This configuration defpends on X11 and, of course, can only be used in
  environments that support X11!  (And it may not even be usable in all of
  those environments without some "tweaking").

  1. Special Framebuffer Configuration

     This configuration uses the same special simulated framebuffer
     configuration options as the nx configuration:

       CONFIG_SIM_X11FB    - Use X11 window for framebuffer
       CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
       CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
       CONFIG_SIM_FBBPP    - Pixel depth in bits

  2. X11 Configuration

     But now, since CONFIG_SIM_X11FB is also selected the following
     definitions are needed

       CONFIG_SIM_FBBPP (must match the resolution of the display).
       CONFIG_FB_CMAP=y

     My system has 24-bit color, but packed into 32-bit words so
     the correct setting of CONFIG_SIM_FBBPP is 32.

  3. X11 Build Issues

     To get the system to compile under various X11 installations
     you may have to modify a few things.  For example, in order
     to find libXext, I had to make the following change under
     Ubuntu 9.09:

       cd /usr/lib/
       sudo ln -s libXext.so.6.4.0 libXext.so

udgram

  This is the same as the nsh configuration except that it includes
  two addition build in applications:  server and client.  These
  applications are provided by the test at apps/examples/udgram.
  This configuration enables local, Unix domain sockets and supports
  the test of the datagram sockets.

  To use the test:

    nsh> server &
    nsh> client

unionfs

  This is a version of NSH dedicated to performing the simple test
  of the Union File System at apps/exmaples/uniofs.  The command
  'unionfs' will mount the Union File System at /mnt/unionfs.  You
  can than compare what you see at /mnt/unionfs with the content
  of the ROMFS file systems at apps/examples/unionfs/atestdir and
  btestdir.

  Here is some sample output from the test:

    NuttShell (NSH)
    nsh> unionfs
    Mounting ROMFS file system 1 at target=/mnt/a with source=/dev/ram4
    Mounting ROMFS file system 2 at target=/mnt/b with source=/dev/ram5
    nsh> ls /mnt/unionfs
    /mnt/unionfs:
     .
     afile.txt
     offset/

   When unionfs was created, file system was joined with and offset called
   offset".  Therefore, all of the file system 2 root contents will appear
   to reside under a directory called offset/ (although there is no
   directory called offset/ on file system 2).  Fie system 1 on the other
   hand does have an actual directory called offset/.  If we list the
   contents of the offset/ directory in the unified file system, we see
   he merged content of the file system 1 offset/ directory and the file
   system 2 root directory:

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

  The directory offset/adir exists on file system 1 and the directory\
  adir/ exists on file system 2.  You can see that these also overlap:

    nsh> ls /mnt/unionfs/offset/adir
    /mnt/unionfs/offset/adir:
     ..
     asubdir/
     adirfile.txt
     bsubdir/
     bdirfile.txt
     .

  The unified directory listing is showing files from both file systems in
  their respective offset adir/ subdirectories.  The file adirfile.txt
  exists in both file system 1 and file system 2 but the version if file
  system 2 is occluded by the version in file system 1.  The only way
  that you can which are looking at is by cat'ing the file:

    nsh> cat /mnt/unionfs/offset/adir/adirfile.txt
    This is a file in directory offset/adir on file system 1

  The file on file system 1 has correctly occluded the file with the same
  name on file system 2.  bdirfile.txt, however, only exists on file
  system 2, so it is not occluded:

    nsh> cat /mnt/unionfs/offset/adir/bdirfile.txt
    This is another file in directory adir on file system 2

  You can see the files in the two file systems before they were unified at
  apps/examples/unionfs/atestdir and btestdir.

userfs

  This is another NSH configuration that includes the built-in application of apps/examples/userfs to support test of the UserFS on the simulation platform.

  To use the test:

    nsh> userfs                 # Mounts the UserFS test file system at
                                # /mnt/ufstest
    nsh> mount                  # Testing is then performed by exercising the
                                # file system from the command line
    nsh> ls -l /mnt/ufstest
    nsh> cat /mnt/ufstest/File1
    etc.

ustream

  This is the same as the nsh configuration except that it includes
  two addition built in applications:  server and client.  These
  applications are provided by the test at apps/examples/ustream.
  This configuration enables local, Unix domain sockets and supports
  the test of the stream sockets.

  To use the test:

    nsh> mount -t binfs /bin
    nsh> server &
    nsh> client

