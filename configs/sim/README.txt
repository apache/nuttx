README
^^^^^^

This README file describes the contents of the build configurations available
for the NuttX "sim" target.  The sim target is a NuttX port that runs as a
user-space program under Linux or Cygwin.  It is a very "low fidelity" embedded
system simulation: This environment does not support any kind of asynchonous
events -- there are nothing like interrupts in this context.  Therefore, there
can be no pre-empting events.

In order to get timed behavior, the system timer "interrupt handler" is called
from the sim target's IDLE loop.  The IDLE runs whenever there is no other task
running.  So, for example, if a task calls sleep(), then that task will suspend
wanting for the time to elapse.  If nothing else is available to run, then the
IDLE loop runs and the timer increments, eventually re-awakening the sleeping task.

Context switching is based on logic similar to setjmp() and longjmp().

The sim target is used primarily as a development and test platform for new
RTOS features.  It is also of academic interest.  But it has no real-world
application that I know of.

NOTE:  In order to facility fast testing, the sim target's IDLE loop, by default,
calls the system "interrupt handler" as fast as possible.  As a result, there
really are no noticeable delays when a task sleeps.  However, the task really does
sleep -- but the time scale is wrong.  If you want behavior that is closer to
normal timing, then you can define CONFIG_SIM_WALLTIME=y in your configuration
file.  This configuration setting will cause the sim target's IDLE loop to delay
on each call so that the system "timer interrupt" is called at a rate approximately
correct for the system timer tick rate.  With this definition in the configuration,
sleep() behavior is more or less normal.

64-Bit Issues
^^^^^^^^^^^^^

As mentioned above, context switching is based on logic like setjmp and longjmp.
This context switching is only available for 32-bit targets.  On 64-bit machines,
this context switching will fail.

There are other 64-bit issues as well.  For example, addresses are retained in
32-bit unsigned integer types in a few places.  On a 64-bit machine, the 32-bit
address storage may correcupt 64-bit addressing.  NOTE:  This is really a bug --
addresses should not be retained in uint32_t types but rather in uintptr_t types
to avoid issues just like this.

The workaround on 64-bit machines for now is to build for a 32-bit target on the
64-bit machine.  This workaround involves modifying the Make.defs file in the
appropriate places so that -m32 is included in the CFLAGS and -m32 and -melf_386
are included in the LDFLAGS.  See the patch 0001-Quick-hacks-to-build-sim-nsh-ostest-on-x86_64-as-32-.patch
that can be found at http://tech.groups.yahoo.com/group/nuttx/files.

Buffered I/O Issues
^^^^^^^^^^^^^^^^^^^

The simulated serial driver has some odd behavior.  It will stall for a long time
on reads when the C stdio buffers are being refilled. This only effects the behavior
of things like fgetc().  Workaround: Set CONFIG_STDIO_BUFFER_SIZE=0, suppressing
all C buffered I/O.

Networking Issues
^^^^^^^^^^^^^^^^^

I never did get networking to work on the sim target.  It tries to use the tap device
(/dev/net/tun) to emulate an Ethernet NIC, but I never got it correctly integrated
with the NuttX networking (I probably should try using raw sockets instead).

X11 Issues
^^^^^^^^^^

There is an X11-based framebuffer driver that you can use exercise the NuttX graphics
subsystem on the simulator (see the sim/nx configuration below).  This may require a
lot of tinkering to get working, depending upon where your X11 installation stores
libraries and header files and how it names libraries.

For example, on UBuntu 9.09, I had to do the following to get a clean build:

    cd /usr/lib/
    sudo ln -s libXext.so.6.4.0 libXext.so

(I also get a segmentation fault at the conclusion of the NX test -- that will need
to get looked into as well).

The X11 examples builds on Cygwin, but does not run.  The last time I tried it,
XOpenDisplay() aborted the program.

Configurations
^^^^^^^^^^^^^^

mount
  Configures to use examples/mount.  This configuration may be
  selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/mount

nettest

  Configures to use examples/nettest.  This configuration
  enables networking using the network TAP device.  It may
  be selected via:

    cd <nuttx-directory>/tools
    ./configure.sh sim/nettest

  NOTES:
  - The NuttX network is not, however, functional on the Linux TAP
    device yet.

  - As of NuttX-5.18, when built on Windows, this test does not try
    to use the TAP device (which is not available on Cygwin anyway), 
    but inside will try to use the Cygwin WPCAP library.  Only the
    most preliminary testing has been performed with the Cygwin WPCAP
    library, however.

    NOTE that the IP address is hard-coded in arch/sim/src/up_wpcap.c.
    You will either need to edit your configuration files to use 10.0.0.1
    on the "target" (CONFIG_EXAMPLE_NETTEST_*) or edit up_wpcap.c to
    select the IP address that you want to use.

nsh
  Configures to use the NuttShell at examples/nsh.  This configuration
  may be selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/nsh

nx
  Configures to use examples/nx.  This configuration may be
  selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/nx

  Special simulated framebuffer configuration options:

  CONFIG_SIM_X11FB    - Use X11 window for framebuffer
  CONFIG_SIM_FBHEIGHT - Height of the framebuffer in pixels
  CONFIG_SIM_FBWIDTH  - Width of the framebuffer in pixels.
  CONFIG_SIM_FBBPP    - Pixel depth in bits

  NOTES:
  - If CONFIG_SIM_X11FB is selected then the following are
    needed

      CONFIG_SIM_FBBPP (must match the resolution of the display).
      CONFIG_FB_CMAP=y

    My system has 24-bit color, but packed into 32-bit words so
    the correct seeting of CONFIG_SIM_FBBPP is 32.

  - For whatever value of CONFIG_SIM_FBBPP is selected, then
    the corresponidng CONFIG_NX_DISABLE_*BPP setting must
    not be disabled.

  - The default in defconfig is to use a generic memory buffer
    for the framebuffer.  defconfig-x11 is an example with X11
    support enabled.  To use this configuration you have to
    configure as follows:

    cd tools
    ./configure.sh sim/nx
    cd ..
    cp configs/sim/nx/defconfig-x11 .config

  - The default is the single-user NX implementation.  To select
    the multi-user NX implementation:

      CONFG_NX_MULTIUSER=y
      CONFIG_DISABLE_MQUEUE=n

  - To get the system to compile under various X11 installations
    you may have to modify a few things.  For example, in order
    to find libXext, I had to make the following change under
    Ubuntu 9.09:

    cd /usr/lib/
    sudo ln -s libXext.so.6.4.0 libXext.so

ostest

  The "standard" NuttX examples/ostest configuration.  This
  configuration may be selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/ostest

pashello

  Configures to use examples/pashello.  This configuration may
  by selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/pashello
