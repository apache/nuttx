.. include:: /substitutions.rst
.. _qemugdb:

=====================================
How to debug NuttX using QEMU and GDB
=====================================

This guide explains the steps needed to use QEMU and GDB to debug
an ARM board (lm3s6965-ek), but it could be modified to work with other
board or architecture supported by QEMU.

Start configuring and compiling the lm3s6965-ek board with qemu-flat profile.

Compiling
=========

#. Configure the lm3s6965-ek

   There is a sample configuration to use lm3s6965-ek on QEMU.

   Just use ``lm3s6965-ek:qemu-flat`` board profile for this purpose. 

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh lm3s6965-ek:qemu-flat

#. Compile

    .. code-block:: console

       $ make -j

Start QEMU
==========

#. You need to start QEMU using the nuttx ELF file just create above:

    .. code-block:: console

       $ qemu-system-arm -M lm3s6965evb -device loader,file=nuttx -serial mon:stdio -nographic -s
       Timer with period zero, disabling
       ABCDF
       telnetd [4:100]
       
       NuttShell (NSH) NuttX-12.0.0
       nsh>

Start GDB to connect to QEMU
============================

   These steps show how to connect GDB to QEMU running NuttX:

    .. code-block:: console

       $ gdb-multiarch nuttx -ex "source tools/gdb/__init__.py" -ex "target remote 127.0.0.1:1234"
        Type "apropos word" to search for commands related to "word"...
        Reading symbols from nuttx...
        set pagination off
        source tools/gdb/lists.py
        source tools/gdb/utils.py
        source tools/gdb/memdump.py

        if use thread command, please don't use 'continue', use 'c' instead !!!
        source tools/gdb/thread.py
        "handle SIGUSR1 "nostop" "pass" "noprint"
        Remote debugging using 127.0.0.1:1234
        0x0000a45a in up_idle () at chip/common/tiva_idle.c:62
        62      }
       (gdb)

#. From (gdb) prompt you can run commands to inpect NuttX:

    .. code-block:: console

        (gdb) info threads
        Id   Thread                Info                                                                             Frame
        *0   Thread 0x2000168c     (Name: Idle_Task, State: Running, Priority: 0, Stack: 1008)                      0xa45a up_idle() at chip/common/tiva_idle.c:62
        1    Thread 0x20005270     (Name: hpwork, State: Waiting,Semaphore, Priority: 224, Stack: 1984)             0xa68c up_switch_context() at common/arm_switchcontext.c:95
        2    Thread 0x20005e30     (Name: nsh_main, State: Waiting,Semaphore, Priority: 100, Stack: 2008)           0xa68c up_switch_context() at common/arm_switchcontext.c:95
        3    Thread 0x20006d48     (Name: NTP_daemon, State: Waiting,Signal, Priority: 100, Stack: 1960)            0xa68c up_switch_context() at common/arm_switchcontext.c:95
        4    Thread 0x20008b60     (Name: telnetd, State: Waiting,Semaphore, Priority: 100, Stack: 2016)            0xa68c up_switch_context() at common/arm_switchcontext.c:95
       (gdb) 

As you can see QEMU and GDB are powerful tools to debug NuttX without using external board or expensive debugging hardware.

