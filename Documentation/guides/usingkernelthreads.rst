====================
Using Kernel Threads
====================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Using+Kernel+Threads


Build Configurations
====================

NuttX can be built in three different configurations: (1) as a FLAT build where 
all of the code resides in a common address space, (2) as a PROTECTED build 
where a memory protection unit (MPU) is used to separate the memory into 
privileged memory for the OS and unprivileged memory for all applications, 
or (3) as a KERNEL build where a memory management unit (MMU) is used place 
the OS in a privileged address space and to place to task (or process) in its 
own virtual address space.

In the last two configurations, applications reside outside of the OS address 
space and in all configurations applications do not have have access to any 
internal resources of the OS.

More information about these build configurations can be found on `the Memory 
Configuration Wiki page <https://cwiki.apache.org/confluence/display/NUTTX/Memory+Configurations>`_.

Thread Types
============

NuttX supports three classes of threads: tasks, pthreads, and kernel threads. 
tasks and pthreads are both application threads and are distinguished by some 
usage semantics and by their hierarchical relationship. tasks are created via 
several different mechanisms: ``task_create()``, ``task_spawn()``, ``execv()``, 
``posix_spawn()``, and others. Tasks may then create pthreads using 
``pthread_create()``.

More information about tasks and pthreads can be found on the 
`NuttX Tasking <https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Tasking>`_ 
Wiki page.

Kernel Threads
==============

Kernel threads are really like tasks except that they run inside the operating 
system and are started with ``kthread_create()`` which is prototyped in 
``include/nuttx/kthread.h``. The differ from tasks in that (1) in PROTECTED and 
KERNEL builds, they have full supervisor privileges, and (2) they have full 
access to all internal OS resources.

In order to build the task into the OS as a kernel thread, you simply have to: 
(1) place the kernel thread code in your board source code directory, and (2) 
start it with ``kthread_create()`` in your board bring-up logic. There a few 
examples of this in the NuttX source tree. Here is one: 
`https://github.com/apache/nuttx/blob/master/boards/arm/stm32/viewtool-stm32f107/src/stm32_highpri.c <https://github.com/apache/nuttx/blob/master/boards/arm/stm32/viewtool-stm32f107/src/stm32_highpri.c>`_

So that is another trick that you can use to architecture optimal solutions: 
Create parts of your applications as kernel threads: They need to reside in 
your board/src directory and the need to be started with ``kthread_create()`` in 
your board bring-up logic. And that is it.
