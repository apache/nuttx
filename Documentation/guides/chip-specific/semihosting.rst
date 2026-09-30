===========
Semihosting
===========

Overview
========

NuttX supports many types of semihosting (syslog, file host sharing, poweroff, etc).
The focus of this document is syslog and file sharing, since these are the most common
features used when porting NuttX to a new platform or when testing requires files from
the host machine (i.e. to play an audio file).

Semihosting SYSLOG
------------------

This feature is highly valuable, particularly during the porting process to a new chip.
It allows the developer to track events and diagnose issues during the early initialization
phase, even if the UART driver support hasn't been implemented. Furthermore, it serves as a
critical debugging tool when a physical serial port isn't available for monitoring.

These are the steps to get semihost syslog working on stm32f4discovery board, but it could
be adapted to other boards as well:

1. Select the stm32f4discovery board with the nsh profile:

.. code-block:: console

    $ ./tools/configure.sh stm32f4discovery:nsh
      Copy files
      Select CONFIG_HOST_LINUX=y
      Refreshing...

2. Run the menuconfig to select the necessary options:

.. code-block:: console

    $ make menuconfig

3. Enable the debug options to be displayed over syslog semihost (we want see the memory allocations) :

.. code-block:: text

    Build Setup  --->
        Debug Options  --->
            [*] Enable Debug Features
            [*]   Enable Error Output (NEW)
            [*]     Enable Warnings Output (NEW)
            [*]       Enable Informational Debug Output (NEW)
            ...
            [*]   Memory Manager Debug Features
            [*]     Memory Manager Error Output 
            [*]     Memory Manager Warnings Output 
            [*]     Memory Manager Informational Output 

4. Enable the semihost syslog support:

.. code-block:: text

    System Type  --->
        [*] Semihosting SYSLOG support

5. We need to disable the /dev/console otherwise the serial initialization will be called:

.. code-block:: text

    RTOS Features  --->
        Files and I/O  --->
            [ ] Enable /dev/console (disable the /dev/console)

6. Disable the serial driver and enable syslog buffer to speed up the output:

.. code-block:: text

    Device Drivers  --->
        [ ] Serial Driver Support  ---- (disable serial driver)
    
        System Logging  ---> 
            [*] Use buffered output

7. Save and leave menuconfig.

8. Compile the firmware:

.. code-block:: console

    $ make -j

9. Flash the firmware in the board:

.. code-block:: console

    $ sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"

10. Start the openocd server to wait for GDB connection:

.. code-block:: console

    $ sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c "reset halt"
    Open On-Chip Debugger 0.12.0
    Licensed under GNU GPL v2
    For bug reports, read
      http://openocd.org/doc/doxygen/bugs.html
    Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
    Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
    Info : clock speed 2000 kHz
    Info : STLINK V2J14S0 (API v2) VID:PID 0483:3748
    Info : Target voltage: 3.202097
    Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
    Info : [stm32f4x.cpu] target has 6 breakpoints, 4 watchpoints
    Info : starting gdb server for stm32f4x.cpu on 3333
    Info : Listening on port 3333 for gdb connections
    [stm32f4x.cpu] halted due to breakpoint, current mode: Thread 
    xPSR: 0x01000000 pc: 0x08004b68 msp: 0x200016b0
    [stm32f4x.cpu] halted due to debug-request, current mode: Thread 
    xPSR: 0x01000000 pc: 0x0800052c msp: 0x200017f0
    Info : Listening on port 6666 for tcl connections
    Info : Listening on port 4444 for telnet connections

11. Open another terminal go to the same directory where you compiled nuttx and run GDB passing the ELF file (nuttx)

.. code-block:: console

    $ gdb-multiarch -ex "set architecture arm" nuttx
    GNU gdb (Ubuntu 15.0.50.20240403-0ubuntu1) 15.0.50.20240403-git
    Copyright (C) 2024 Free Software Foundation, Inc.
    ...
    Reading symbols from nuttx...
    The target architecture is set to "arm".
    
    (gdb) target extended-remote 127.0.0.1:3333
    Remote debugging using 127.0.0.1:3333
    0x0800052c in start ()
    
    (gdb) monitor arm semihosting enable
    semihosting is enabled
    
    (gdb) monitor arm semihosting_fileio enable
    semihosting fileio is enabled
    
    (gdb) monitor reset halt
    [stm32f4x.cpu] halted due to debug-request, current mode: Thread 
    xPSR: 0x01000000 pc: 0x0800052c msp: 0x200017f0, semihosting fileio
    
    (gdb) c
    Continuing.
    mm_initialize: Heap: name=Umem, start=0x200017f0 size=124944
    mm_addregion: [Umem] Region 1: base=0x20001960 size=124576
    mm_malloc: Allocated 0x20001970, size 72
    mm_malloc: Allocated 0x200019b8, size 40
    mm_addregion: [Umem] Region 2: base=0x10000000 size=65536
    mm_malloc: Allocated 0x10000010, size 48
    mm_malloc: Allocated 0x10000040, size 48
    mm_malloc: Allocated 0x10000070, size 48
    mm_malloc: Allocated 0x100000a0, size 24
    mm_malloc: Allocated 0x100000b8, size 16
    mm_malloc: Allocated 0x100000c8, size 32
    mm_malloc: Allocated 0x100000e8, size 208
    mm_malloc: Allocated 0x100001b8, size 2056
    mm_malloc: Allocated 0x100009c0, size 48
    mm_malloc: Allocated 0x100009f0, size 896
    mm_malloc: Allocated 0x10000d70, size 32
    mm_malloc: Allocated 0x10000d90, size 16
    mm_malloc: Allocated 0x10000da0, size 2056
    mm_free: Freeing 0x100001b8
    mm_free: Freeing 0x100000e8
    mm_malloc: Allocated 0x100000e8, size 768
    mm_free: Freeing 0x100000e8
    mm_free: Freeing 0x10000d90
    mm_free: Freeing 0x10000d70
    mm_free: Freeing 0x10000da0
    mm_free: Freeing 0x100009f0

Semihosting files
-----------------

Relevant files:

.. code-block:: bash

    fs/hostfs/
    arch/arm/include/armv7-m/syscall.h
    arch/arm/src/common/up_hostfs.c

Mounting:

.. code-block:: bash

    mount -t hostfs -o fs=/host/path /local/path
