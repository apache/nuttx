.. _nuttx-initialization-sequence:

=============================
NuttX Initialization Sequence
=============================
 
Overview
========

This initialization sequence is really quite simple because the system runs
in single-thread mode up until the point the that is starts the application.
That means the initialization sequence is just a simple, straight-line
of function calls.
It is until just before starting the application that system goes to
multi-threaded mode and things can get more complex.

At the highest level, the NuttX initialization sequence can be
represented in three phases:

* **Phase A** - The hardware-specific power-on reset initialization,

* **Phase B** - NuttX RTOS initialization, and

* **Phase C**  - Application Initialization.

Each of these will be discussed in more detail in the following sections.


Case example: STM32 F4
======================

In this discussion, we'll use the STM32 F4 MCU and its popular evaluation
board STM32F4Discovery as example but the explanation can be applied
to any supported architecture.

Here is the map of initialization function calls::

  __start()-arch/arm/src/stm32/stm32_start.c
      |
      +--*Set stack limit
      +--stm32_clockconfig()
      +--stm32_fpuconfig()
      +--stm32_lowsetup()
      +--stm32_gpioinit()
      +--showprogress('A')
      +--
      +--
      +--stm32_boardinitialize()-boards/arm/stm32/stm32f4discovery/src/stm32_boot.c
      |    |
      |    +--stm32_spidev_initialize()-stm32_spi.c:ONLY CHIP SELECTS
      |    +--stm32_usbinitialize()-
      |    +--stm32_netinitialize()-
      |    +--board_autoled_initialize()-
      |                  
  nx_start()-sched/init/nx_start.c
      |                  
      +--*Initialize global data structures
      +--*Initialize OS facilities
      +--net_initialize()-net/net_initialize.c
      |    |   
      |    +--net_lockinitialize()
      |    +--mld_initialize()
      |    +--can_initialize()
      |    +--netlink_initialize()
      |    +--tcp_initialize()
      |    +--udp_initialize()
      |    +--usrsock_initialize()
      |
      +--up_initialize()-arch/arm/src/common/up_initialize.c
      |    |                  
      |    +--arm_dmainitialize()
      |    +--Config basic /dev nodes
      |    +--arm_serialinit()
      |    +--Console Init
      |    +--Crypto Config
      |    +--arm_netinitialize()
      |    |    |
      |    |    +--stm32_spibus_initialize()
      |    |
      |    |
      |    +--arm_usbinitialize()
      |    +--L2 Cache Init
      |
      +--board_early_initialize()
      +--g_nx_initstate = OSINIT_HARDWARE
      +--shm_initialize()
      +--lib_initialize()
      +--binfmt_initialize()
      +--Start SMP
      +--syslog_initialize()
      +--g_nx_initstate = OSINIT_OSREADY
      +--DEBUGVERIFY(nx_bringup())-sched/init/nx_bringup.c
      |    |
      |    +--nx_pgworker()
      |    +--nx_workqueues()
      |    +--nx_create_initthread()-sched/init/nx_bringup.c
      |         |              
      |         +----+different thread
      |         :    :
      |         :  nx_start_task()-sched/init/nx_bringup.c
      |         :    :        
      |     same+----+--nx_start_application()-sched/init/nx_bringup.c
      |     thread        |
      |                   +--board_late_initialize()-stm32_boot.c:BOARD_DEPENDANT
      |                   |    |
      |                   |    +--stm32_bringup()
      |                   |         |
      |                   |         +--stm32_i2ctool()
      |                   |         +--board_bmp180_initialize()
      |                   |         +--stm32_sdio_initialize()
      |                   |         +--stm32_usbhost_initialize()
      |                   |         +--stm32_pwm_setup()
      |                   |         +--stm32_can_setup()
      |                   |         +--etc
      |                   |
      |                   +--
      |
      |
      |
      |
      |
      +--kmm_givesemaphore()
      +--up_idle()


Phase A - Power-On Reset Initialization
=======================================

The system begins execution when the processor is reset.
This usually at power-on, but all resets are basically the same whether
they occur because of power-on, pressing the reset button, or on a watchdog
timer expiration.

.. note:: The code that executes when the processor is reset is unique
          to the particular CPU architecture and is not a common part
          of NuttX.

The kind of things that must be done by the architecture-specific reset
handling includes:

* Putting the processor in its operational state. This may include things
  like setting CPU modes; initializing co-processors, etc.
* Setting up clocking so that the software and peripherals operate as expected.
* Setting up the C stack pointer (and other processor registers).
* Initializing memory.
* Starting NuttX.


Memory Initialization
---------------------

In C implementations, there are two general classes of variable storage.
First there are the initialized variables.
For example, consider the global variable x:

.. code-block:: c

  int x = 5;

The C code must be assured that after reset, the variable x has the value 5.
Initialized variable of this kind are retained in a special memory section
called data (or ``.data``).

Other variables are not initialized. Like the global variable y:

.. code-block:: c

  int y;

But the C code will still expect y to have an initial value.
That initial value will be zero.
All uninitialized variables of this type need to have the value zero.
These uninitialized variables are retained in a section called bss
(or ``.bss``).

When we say that the reset handling logic initializes memory, we mean two things:

1. It provides the (initial) values of the initialized variables by copying
   the values from FLASH into the ``.data`` section, and
2. It resets all of the uninitialized variables to zero.
   It clears the ``.bss`` section.

Lets walk through reset sequence. This reset logic can be found in two files::

    nuttx/arch/arm/src/stm32_vectors.S
    nuttx/arch/arm/src/stm32_start.c


nuttx/arch/arm/src/stm32_vectors.S
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This file provides all of the STM32 exception vectors and power-on reset
is simply another exception vector.
There are few important things to note about this file.

``.section .vectors, ax``. This pseudo operation will place all of the
vectors into a special section call ``.vectors``.
On of the STM32 F4 linker scripts is located at
``nuttx/boards/arm/stm32/stm32f4discovery/scripts/ld.script``.
In that file, you can see that section ``.vectors`` is forced to lie
at the very beginning of FLASH memory.
The STM32 F4 can be configured to boot in different ways via strapping.
If it is strapped to boot from FLASH, then the STM32 FLASH memory will
be aliased to address ``0x0000 0000`` when the reset occurs.
That is the address of the power-up reset interrupt vector.

The first two 32-bit entries in the vector table represent the power-up
exception vector (which we know will be positioned at address
``0x0000 0000`` when the reset occurs). Those two entries are:

.. code-block:: c

  .word IDLE_STACK /* Vector  0: Reset stack pointer */
  .word __start    /* Vector  1: Reset vector */

The Cortex-M family is unique in the way that is handles the reset vector.
Notice that there are two values: the stack pointer for the start-up thread
(the IDLE thread), and the entry point in the IDLE thread.

When the reset occurs, the the stack pointer is automatically set
to the first value and then the processor jumps to reset entry point
``__start`` specified in the second entry.

This means that the reset exception handling code can be implemented in C
rather than assembly language.


nuttx/arch/arm/src/stm32_start.c
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The reset vector ``__start`` lies in the file
``nuttx/arch/arm/src/stm32/stm32_start.c`` and does the real,
low-level architecture-specific initialization. This initialization includes:

1. ``stm32_clockconfig()`` - Initialize the PLLs and peripheral clocking
   needed by the board.

2. ``stm32_fpuconfig()`` - If the STM32 F4's hardware floating point
   is initialized, then configure the FPU and enable access to the FPU
   co-processors.

3. ``stm32_lowsetup()`` - Enable the low-level UART. This is done very early
   in initialization so that we can get serial debug output to the console
   as soon as possible. If you are doing a board bring-up this
   is very important.

4. ``stm32_gpioinit()`` - Perform any GPIO remapping that is needed
   (this is a stub for the F4, but the F1 family requires this step).

5. ``showprogress('A')`` - This simply outputs the character ``A`` on the
   serial console (only if ``CONFIG_DEBUG`` is enabled). If debug is enabled,
   you will always see the letters ABDE output on the console. That output
   all comes from this file.

6. Next the memory is initialized:

   a. The ``.bss`` section is set to zero (Letter ``B`` is then output if
      ``CONFIG_DEBUG`` is enabled), then
   b. The ``.data`` section is set to its initial values (The letter ``C``
      is output if debug is enabled),

7. ``stm32_boardinitialize()`` - Board-specific logic is initialized by
   calling this function. For the case of the STM32F4Discovery board,
   this logic can be found at
   ``nuttx/boards/arm/stm32/stm32f4discovery/src/stm32_boot.c`` and does
   the following operations:

   a. ``stm32_spidev_initialize()`` - Initialize SPI chip selects
      if SPI is enabled.
   b. ``stm32_usbinitialize()`` - Initialize hardware USB devices if enabled.
   c. ``stm32_netinitialize()`` - Initialize hardware network devices
      if enabled.
   d. ``board_autoled_initialize()`` - Configure on-board LEDs
      if LED support has been selected.

8. When ``stm32_boardinitialize()`` returns to ``__start()``, the low-level,
architecture-specific initialization is complete.


Phase B - NuttX RTOS Initialization
===================================

``nx_start()``
--------------

This function resides in the file ``nuttx/sched/init/nx_start.c`` and
is the NuttX entry point.

It is called by ``__start()`` and performs the next phase of RTOS-specific
initialization before bringing up the application.

The operations performed by ``nx_start()`` are summarized below.
Note that many of these features can be disabled from the NuttX configuration
file and in that case those operations are not performed:

1. Initializes some NuttX global data structures,

2. Initializes the TCB for the IDLE (i.e, the thread that the initialization
   is performed on),

3. ``nxsem_initialize()`` - Initialize the POSIX semaphore facilities.
   This needs to be done first because almost all other OS features depend
   on POSIX counting semaphores.

4. Memory organization - This includes heap configuration, memory manager,
   paging, I/O buffers, etc.

5. ``task_initialize()`` - Initialize task data structures.

6. ``fs_initialize()`` - Initialize the file system (needed to support
   device drivers).

7. ``irq_initialize()`` - Initialize the interrupt handler subsystem.
   This initializes only data structures; CPU interrupts are still disabled.

8. ``wd_initialize()`` - Initialize the NuttX watchdog timer facility,

9. ``clock_initialize()`` - Initialize the system clock,

10. ``timer_initialize()`` - Initialize the POSIX timer facilities,

11. ``nxsig_initialize()`` - Initialize the POSIX signal facilities,

12. ``nxmq_initialize()`` - Initialize the POSIX message queue facilities,

13. ``pthread_initialize()`` - Initialize the POSIX pthread facilities,

14. ``net_initialize()`` - Initialize networking facilities,

.. note:: Up to this point, all of the initialization steps have only been
          software initializations. Nothing has interacted with the hardware.
          Rather, all of these steps simply prepared the environment so that
          things like interrupts and threads can function properly.
          The next phases depend upon that setup.

15. ``up_initialize()`` - The processor specific details of running
    the operating system will be handled here. Such things as setting up
    interrupt service routines and starting the clock are some of the things
    that are different for each processor and hardware platform.

    All ARM-based MCUs share a common ``up_initialize()`` implementation
    provided at ``nuttx/arch/arm/src/common/up_initialize.c``. The operations
    perform by this common ARM initialization will, however, call into
    facilities provided by the particular ARM chip.
    For the STM32 F4, those facilities would be provided by logic in files
    as ``nuttx/arch/arm/src/stm32``. The common ARM initialization sequence is:

    * ``up_color_intstack()`` - Colorize the interrupt stack.

    * ``arm_addregion()`` - The basic heap was set up during processing by
      ``nx_start()``. However, if the board supports multiple, discontiguous
      memory regions, any addition memory regions can be added to the heap
      by this function. For the STM32 F4, ``up_addregion()`` is implemented
      in ``nuttx/arch/arm/src/stm32/stm32_allocateheap.c``.

    * ``arm_pminitialize()`` - If ``CONFIG_PM`` is defined, the function must
      initialize the power management subsystem. This MCU-specific function
      must be called very early in the intialization sequence before any other
      device drivers are initialized (since they may attempt to register with
      the power management subsystem). There is no implementation
      of ``up_pminitialize()`` for any STM32 platform.

    * ``arm_dmainitialize()`` - Initialize the DMA subsystem.
      For the STM32 F4, this DMA initialization can be found in
      ``nuttx/arch/arm/src/stm32/stm32_dma.c`` (which includes
      ``nuttx/arch/arm/src/stm32f4xxx_dma.c``).

    * ``devnull_register()`` - Registers the standard ``/dev/null``.

    * ``devrandom_register()`` - Registers the standard ``/dev/random``.

    * ``devurandom_register()`` - Registers the standard ``/dev/urandom``.

    * ``devzero_register()`` - Registers the standard ``/dev/zero``.

    * ``loop_register()`` - Registers the standard ``/dev/loop``.

    * ``note_register()`` - Registers the standard ``/dev/note``.

    * ``arm_serialinit()`` - Initialize the **standard** serial driver
      (found at ``nuttx/arch/arm/src/stm32/stm32_serial.c`` STM32 F4).

    * ``arm_netinitialize()`` - Initialize the network. For the STM32 F4,
      this function is in ``nuttx/arch/arm/src/stm32/stm32_eth.c``.

    * ``arm_usbinitialize()`` - Initialize USB (host or device).
      For the STM32 F4, this function is in
      ``nuttx/arch/arm/src/stm32/stm32_otgfsdev.c``.

    * ``arm_l2ccinitialize()`` - Initialize the L2 cache if present
      and selected.

    * ``up_ledon(LED_IRQSENABLED)`` - Finally, ``up_initialize()``
      illuminates board-specific LEDs to indicate the IRQs are now enabled.


16. ``board_early_initialize()`` - If ``CONFIG_BOARD_EARLY_INITIALIZE`` is
    selected, then an additional initialization call will be performed in the
    boot-up sequence to a function called ``board_early_initialize()``.
    It will be called immediately after ``up_initialize()`` (and may be
    thought of as a board-specific, extension of ``up_initialize()``)
    and well ``before board_late_initialize()`` is called and the initial
    application is started.

17. ``g_nx_initstate = OSINIT_HARDWARE`` - This signals that basic
    hardware setup is complete.

18. ``shm_initialize()`` - Initialize shared memory support.

19. ``lib_initialize()`` - Initialize the C libraries. This is done last
    because the libraries may depend on the above.

20. ``binfmt_initialize()`` - Initialize binary loader subsystem.

21. Start SMP support in multi-core MCUs. This is not the case
    in STM32F4Discovery but relevant because here are created ``stdin``,
    ``stdout`` and ``stderr`` for each CPU's (even if there is only one)
    IDLE task. All tasks subsequently created by the IDLE thread will inherit
    these file descriptors.

22. ``syslog_initialize()`` - Late initialization of the system logging
    device. Some SYSLOG channel must be initialized late in the initialization
    sequence because it may depend on having IDLE task file structures setup.

23. ``nx_bringup()`` - Create the initial tasks. This will be described
    in more detail below.

``nx_bringup()``
----------------

This function is called at the very end of the initialization sequence
in ``nx_start()``, just before entering the IDLE loop. It is located
in ``nuttx/sched/init/nx_bringup.c`` and it starts all of the required
threads and tasks needed to bring up the system.

This function performed the following specific operations:

* ``nx_pgworker()`` - Start the page fill worker kernel thread that will
  resolve page faults. This should always be the first thread started because
  it may have to resolve page faults in other threads. This is the task that
  runs in order to satisfy page faults in processors that have an MMU and
  in configurations where on-demand paging is enabled.

* ``nx_workqueues()`` - Start the worker thread. The worker thread may be used
  to execute any processing deferred to the worker thread via APIs provided
  in ``include/nuttx/wqueue.h``. The worker thread's primary function is
  as the “bottom half” for extended device driver processing but can be used
  for a variety of purposes like misc garbage clean-up.

* ``nx_create_initthread()`` - Once the operating system has been initialized,
  this funcions either directly calls ``nx_start_application()`` or creates
  a thread for running it

* ``nx_start_application()`` - If set in the NuttX configuration,
  this function calls ``board_late_initialize()``.

  * ``board_late_initialize()`` is a last-minute, board-specific
    initialization. Note that there was earlier, board-specific initialization
    calls (to ``stm32_board_initialize()`` and
    to ``board_early_initialize()``). The difference here is these first,
    low-level initialization calls were made before the OS was completely
    launched. ``board_late_initialize()``, on the other hand, is called
    at the end after the OS has been initialized but before any application
    tasks have been started.
    ``board_late_initialize()`` would be an ideal place to do board-specific
    initialization steps that depend on having a fully initialized OS
    such as memory allocations, initialization of complex device drivers,
    mounting of file systems, etc.

  * After that, ``nx_start_application()`` launches the application either
    by creating a task for it or executing a program from a filesystem after
    mounting it.

  * In the case of creating a task for the application, its entry has
    the name ``user_start()``. ``user_start()`` is provided by application
    code and when it runs, it begins the application-specific phase of the
    initialization sequence as described below.

.. note:: The default ``user_start()`` entry point can be changed to use
          one of the named applications used by NSH. This is a start-up option
          that is not often used and will not be discussed further here.

And finally enter the IDLE loop. After completing the initialization, the role
of the IDLE thread changes. It becomes the thread that executes only when
there is nothing else to do in the system (hence, the name IDLE thread).

IDLE Thread Activities
----------------------

As mention, the IDLE thread is the thread that executes only when there is
nothing else to do in the system. It has the lowest priority in the system.
It always has the priority 0. It is the only thread that is permitted to have
the priority 0. And it can never be blocked (otherwise, what would run then?).

As a result, the IDLE thread is always in the ``g_readytorun`` list and,
in fact, since that list is prioritized, can guaranteed to always be
the final entry at the tail of the ``g_readytorun`` list.

The IDLE is an an infinite loop. But this does not make it a “CPU hog”.
Since it is the lowest priority, the it can be suspended whenever anything
else needs to run.

The IDLE thread does two things in this infinite loop:

1. If the worker was not started (see ``nx_bringup()`` below), then the IDLE
   thread will perform memory clean-up. Memory clean is required to handle
   deferred memory deallocation. Memory allocations must be deferred when
   the memory is freed in a context where the software does not have access
   to the heap and, hence, cannot truly free the memory (such as
   in an interrupt handler). In this case, the memory is simply put into
   a list of freed memory and, eventually, cleaned up by the IDLE thread.

.. note:: The worker thread's primary function is as the “bottom half”
          for extended device driver processing. If the worker thread
          was started, then it will run at a higher priority than the
          IDLE thread. In this case, the worker thread will take over
          responsibility for cleaning up these deferred allocations.

2. Then the loop calls ``up_idle()``. The operations performed
   by ``up_idle()`` are architecture- and board-specific. In general,
   this is the location where CPU-specific reduced power operations
   may be performed.

STM32 F4 IDLE thread
--------------------

The default STM32 F4 IDLE thread is located at
``nuttx/arch/arm/src/stm32_idle.c``.
This default version does very little:

1. It includes a example, “skeleton” function that illustrates that kinds
   of things that you can do if ``CONFIG_PM`` is enabled (this example code
   is not fully implemented in the default IDLE logic).
2. The it executes the Cortex-M thumb2 instruction wfi which causes the CPU
   to sleep until the next interrupt occurs.


Phase C - Application Initialization
====================================

At the conclusion of the OS initialization phase in ``nx_start()``,
the user application is started by creating a new task
at the entry point ``user_start()``.

There must be exactly one entry point called ``user_start()``
in every application built on top of NuttX.

Any additional initialization performed in the ``user_start()`` function
is purely application dependent.


A Simple Hello World Application
--------------------------------

The simplest user application would be the “Hello, World!” example.
See ``apps/examples/hello``. Here is the whole example:

.. code-block:: c

  int user_start(int argc, char *argv[])
  {
    printf("Hello, World!!\n");
    return 0;
  }

In this case, no additional application initialization is needed.
It just “says hello” and exits.

A Nutt Shell User Application/Command
-------------------------------------

The NuttShell (NSH) is a simple shell application that may be used with NuttX.
It is described here. It supports a variety of commands and is (very) loosely
based on the bash shell and the common utilities used
in Unix shell programming.

NSH is implemented as a library that can be found at ``nuttx-apps/nshlib``.
The NSH start-up sequence is very simple. As an example, the the code at
``apps/examples/nsh/nsh_main.c`` illustrates how to start NuttX.
It simple does the following:

1. If you have C++ static initializers, it will call your implementation
   of ``up_cxxinitialize()`` which will, in turn, call those
   static initializers.
2. This function then calls ``nsh_initialize()`` which initializes
   the NSH library. ``nsh_initialize()`` is described in more detail below.
3. If the Telnet console is enabled, it calls ``nsh_telnetstart()`` which
   resides in the NSH library. ``nsh_telnetstart()`` will start the Telnet
   daemon that will listen for Telnet connections and start remote
   NSH sessions.
4. If a local console is enabled (probably on a serial port), then
   ``nsh_consolemain()`` is called. ``nsh_consolemain()`` also resides
   in the NSH library. ``nsh_consolemain()`` does not return so that
   finished the entire NSH initialization sequence.

``nsh_initialize()``
^^^^^^^^^^^^^^^^^^^^

The NSH initialization function, ``nsh_initialize()``, be found in
``apps/nshlib/nsh_init.c``. It does only three things:

``nsh_romfsetc()`` if so configured, it executes an NSH start-up script that
can be found at ``/etc/init.d/rcS`` in the target file system.
``/etc`` is the location where a read-only, ROMFS file system is mounted by
``nsh_romfsetc()``. The ROMFS image is, itself, just built into the firmware.
By default, this rcS startup script contains the following logic:

.. code-block:: sh

  # Create a RAMDISK and mount it at XXXRDMOUNTPOUNTXXX

  mkrd -m XXXMKRDMINORXXX -s XXMKRDSECTORSIZEXXX XXMKRDBLOCKSXXX
  mkfatfs /dev/ramXXXMKRDMINORXXX
  mount -t vfat /dev/ramXXXMKRDMINORXXX XXXRDMOUNTPOUNTXXX

Where the ``XXXX*XXXX`` strings get replaced in the template
when the ROMFS image is created:

* ``XXXMKRDMINORXXX`` will become the RAM device minor number. Default: ``0``.
* ``XXMKRDSECTORSIZEXXX`` will become the RAM device sector size.
* ``XXMKRDBLOCKSXXX`` will become the number of sectors in the device.
* ``XXXRDMOUNTPOUNTXXX`` will become the configured mount point.
  Default: ``/etc``.

This script will, then, create a RAMDISK, format a FAT file system
on the RAM disk, and then mount the FAT filesystem at a configured mountpoint.
This rcS template file can be found at ``apps/nshlib/rcS.template``.
The resulting ROMFS file system can be found in
``apps/nshlib/nsh_romfsimg.h``.

* ``boardctl()`` - Next any architecture-specific NSH initialization will be
  performed (if any). The NSH initialization logic will all the non-standard
  OS interface ``boardctl()`` like: ``(void)boardctl(BOARDIOC_INIT, 0);``.
  The first argument, the command ``BOARDIOC_INIT``, indicates that the
  ``boardctl()`` is being requested to perform application-oriented
  initialization. In response to this command, ``boardctl()`` will call the
  board-specific implementation of ``board_app_initialize()``.
  That function is not generally available to application level code in
  all configurations but can always be accesses via ``boardctl()``.

* ``board_app_initialize()`` - For the STM32F4Discovery, this architecture
  specific initialization can be found at
  ``boards/arm/stm32/stm32f4discovery/src/stm32_appinit.c``.
  This it does things like:

  1. Initialize SPI devices.
  2. Initialize SDIO.
  3. Mount any SD cards that may be inserted.


There are two possibilities then for application startup initialization:

1. In the application itself via ``boardctl(BOARDIOC_INIT, 0)``, OR
2. in the OS using ``board_late_initialize()``.

Each works well in certain contexts but there are also things that I don't
like about either possibility.

``boardctl()``
--------------

You can see this application-controlled initialization in, for example,
the NSH application in `nuttx-apps/nshlib``. This is seen as the call to
``boardctl(BOARDIOC_INIT, 0)``. This requires that the ``boardctl()``
interface be enabled with ``CONFIG_LIB_BOARDCTL``.

When ``CONFIG_LIB_BOARDCTL=y``, board-specific OS internal logic must provide
the interface ``board_app_initilize()``. ``boardctl(BOARDIOC_INIT, 0)`` is the
application level interface that provides the proper interface to
``board_app_initilize()``. All board-specific, driver-level initialization may
then be performed in ``board_app_initilize()`` under the control
of the application.

With ``CONFIG_LIB_BOARDCTL=y``, there are two levels of application-specific
initialization:
(a) an OS/kernel level application initialization that needs to be performed
before the application is started, and
(b) a user/application level initialization that can be performed after
the application has started.

``board_late_nitialize()``
--------------------------

``board_late_initialize()`` is the OS/kernel level application initialization
that is performed before the application has started. It usually works
quite well and is usually a good alternative for ``boardctl(BOARDIOC_INIT)``.
To avoid issues with trying to perform initialization on the IDLE thread,
``board_late_initialize()`` is called from an internal kernel thread.

The IDLE thread has limitations: It cannot wait for events so
it can only be used for simple, straight-line initialization logic.
That may be insufficient in some cases.
For this reason, ``board_late_initialize()`` must run on a kernel thread.

Consider the ``board_late_initialize()`` calling sequence:
``board_late_initialize()`` is called directly from ``do_app_start()`` in
``nuttx/sched/init/nx_bringup.c`` just before starting the application task.
``nx_create_initthread()`` is, in turn, called from the function
``nx_start_application()``. If ``CONFIG_BOARD_LATE_INITIALIZE`` is
not defined, then this is just a normal C function call.
But if ``CONFIG_BOARD_LATE_INITIALIZE`` is defined, then an intermediate,
trampoline kernel thread is started. That kernel thread executes
``do_app_start()`` and moves the initialization off of the IDLE thread.
This works great but is a little more complex than I would like.

``apps/platform``
-----------------

There is also special place for user/application initialization.
That is the ``nuttx-apps/platform`` platform directory. This directory
should be a mirror of the ``nuttx/configs`` directory.
There should be a board directory in ``nuttx-apps/platform`` for every
board that is in ``nuttx/configs``.
