.. _build-modes-detail:

=================================
Flat, Protected and Kernel Builds
=================================

.. _flat-build:

Flat, Embedded Build
====================

The normal build of NuttX for the typical embedded environment uses
a single blob of code in a flat address space.
For most lower end CPUs (such as the ARM Cortex-M family),
this means executing directly out of the physical address space.

Even if the CPU has an MMU (such as with the ARM Cortex-A family),
the typical NuttX build still uses a flat address space with the MMU
providing only an identity mapping.

In this case, there is still benefit from using the MMU because
the MMU provides fine control over caching and memory behavior
over the address space.


.. _on-demand-paging:

On-Demand Paging
================

NuttX also supports on-demand paging via ``CONFIG_PAGING``.
On-demand paging is a method of virtual memory management and requires
the the CPU architecture support a MMU.

In a system that uses on-demand paging, the OS responds to a page fault
by copying data from some storage media into physical memory and setting up
the MMU to provided the necessary virtual address mapping.
The CPU can then continue from the page fault with the necessary memory
in place for the virtual address.

Execution Image
---------------

The execution image is still built as one blob and appears as one blob on the
storage media. But the execution image is paged into arbitrary physical
addresses with non-contiguous virtual addresses.
The physical and virtual address spaces are then "checker boards"
of memory in use.

Advantages
----------

The main advantage of on-demand paging is that you can execute a single
program that is much larger than the physical address space or a collection
of programs that together are much larger than the physical address space.

Current Implementation
----------------------

On-demand paging is currently implemented only for the NXP LPC31xx family.
The LPC31xx has a 192KiB internal SRAM and with on-demand paging the LPC31xx
can execute a huge program residing in SerialFLASH by bringing in new pages
as needed from the SerialFLASH when ``_page`` ``fault_s`` occur.


.. _protected-build:

Protected Build
===============

Protected Build Mode
--------------------

NuttX also supports a protected build mode for certain CPU architectures
if ``CONFIG_BUILD_PROTECTED`` is selected.

**In this mode, NuttX is built as two blobs, one privileged and
one unprivileged.** The privileged blob contains the RTOS, and the other,
unprivileged blob holds all of the applications.

The build supports system calls via a call gate so that the unprivileged,
application code can access the privileged RTOS services.

Memory Protection
-----------------

Within each blob, the address space is flat. No MMU is required to support
the protected build since no address mapping is performed.

In fact, this feature is currently available only for the ARM Cortex-M family.
In this case the Cortex-M's MPU provides the security in the address spaces
of the two blobs.

This feature could also be implemented with a CPU that supports an MMU,
but there has thus far been no reason to implement such a configuration.

Dynamic Memory Allocation
-------------------------

The purpose of protected build then is focused primarily on securing the OS
and CPU resources from potential rogue applications.

The MPU simply protects the hardware, code regions, and data regions
of the RTOS. But dynamic memory allocations become more complex.
Protection is also required for (certain) memory allocations made by the RTOS.
The RTOS must also be capable of allocating memory that is accessible
by user applications (such as the user thread stacks).

Dual Heaps
----------

In systems with MMUs, the privilege of each page of memory can be controlled
and there are established architectures for memory management of processes
(see below). However, with only an MPU with a limited number of pages
(the Cortex-M has 8 pages only!) we are forced to resolve this problem
by dividing available heap memory into two heaps:
a privileged heap and an unprivileged heap, using different allocations
mechanisms for each (kmalloc and malloc, respectively).


.. _addrenv:

Address Environments
====================

If the option ``CONFIG_ARCH_ADDRENV`` is selected, then NuttX will support
address environments in the following way: the base code is still one blob
and identical in every way to the "Flat Embedded Build" discussed above.
But all applications are loaded into RAM from executable files,
separately compiled and separately linked programs, that reside
in a file system.

Instead of starting the user application at a fixed, in-memory address
(such as ``nsh_main()``), the system will start the program contained
in an executable file, given its path.

That initial user program can then start additional applications
from executable files in a file system.

Per Program
-----------

As each program is started, a new address environment is created
for the new task. This address environment is then unique for each task.
A task does not have the capability to access just anything in the address
environment. A task may only access addresses within its own address space
and within the address space of the base code.

MMU (Memory Management Unit)
----------------------------

The CPU must support an MMU in order to provide address environments.

This feature was originally implemented to support the ZiLOG Z180 which
is an 8-bit CPU (basically a Z80) that also supports a simple MMU.
Specifically for the P112 platform. Unfortunately, due to complex tool
issues and fading interest, that port was fully implemented but never tested.

As of this writing, the implementation of address environment support
for the Cortex-A family is complete and verified.
An example configuration is available at
``nuttx/boards/arm/sama5/sama5d4-ek/configs/elf``.


.. _kernel-build:

Kernel Build
============

The Kernel Build
----------------

And finally, there is the kernel build that is enabled with
``CONFIG_BUILD_KERNEL=y``.
The NuttX kernel build mode is similar to building with address environments:

* Each application process executes from its own, private address environment.

But, in addition, there are some features similar to the protected build mode:

* NuttX is built as a monolithic kernel, similar to the way that NuttX
  is built in the Protected Build Mode.
* All of the code that executes within the kernel executes in privileged,
  kernel mode. Again, this is analogous to the Protected Build Mode.
* All user applications are executed with their own private
  address environments in unprivileged, user-mode.

MMU Required
------------

In order to support this kernel build mode, the **processor must provide
a Memory Management Unit (MMU)**. The MMU is used to provide both the address
environment for the user application as well as to enforce
the user-/kernel-mode privileges.

This kernel build feature has been fully implemented and verified
on the Cortex-A family of processes.
A functioning example can be found at
``nuttx/boards/arm/sama5/sama5d4-ek/configs/knsh``.

Process Environment
-------------------

Such user applications that execute their own private, unprivileged address
environments are usually referred to as processes.

The ``CONFIG_BUILD_KERNEL=y`` build is the first step toward support
for processes in NuttX.
