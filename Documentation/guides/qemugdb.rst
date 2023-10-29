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

       $ gdb-multiarch -ix tools/nuttx-gdbinit nuttx
       (gdb) target extended-remote localhost:1234
       Remote debugging using localhost:1234
       0x000012ee in up_mdelay (milliseconds=milliseconds@entry=250)
           at common/arm_mdelay.c:51
       51	      for (j = 0; j < CONFIG_BOARD_LOOPSPERMSEC; j++)
       (gdb)

#. From (gdb) prompt you can run commands to inpect NuttX:

    .. code-block:: console

       (gdb) info_nxthreads
       target examined
       _target_arch.name=armv7
       $_target_has_fpu : 1
       $_target_has_smp : 0
       saved current_tcb (pid=0)
       * 
       0 Thread 0x20001538  (Name: Idle Task, State: Running, Priority: 0, Stack: 464/1000) PC: 0x12fc in up_mdelay()
       saved current_tcb (pid=0)
       
       1 Thread 0x20005060  (Name: hpwork, State: Waiting,Semaphore, Priority: 224, Stack: 320/1992) PC: 0x47dd in work_thread()
       saved current_tcb (pid=0)
       
       2 Thread 0x20005c30  (Name: nsh_main, State: Waiting,Semaphore, Priority: 100, Stack: 1016/2000) PC: 0x1 in _vectors()
       saved current_tcb (pid=0)
       
       3 Thread 0x20006b40  (Name: NTP daemon, State: Waiting,Signal, Priority: 100, Stack: 864/1952) PC: 0x0 in _vectors()
       saved current_tcb (pid=0)
       
       4 Thread 0x20008540  (Name: telnetd, State: Waiting,Semaphore, Priority: 100, Stack: 616/2008) PC: 0x20008fd4 in No()
       saved current_tcb (pid=0)
       saved current_tcb (pid=0)
       saved current_tcb (pid=0)
       saved current_tcb (pid=0)
       (gdb) 

As you can see QEMU and GDB are powerful tools to debug NuttX without using external board or expensive debugging hardware.

