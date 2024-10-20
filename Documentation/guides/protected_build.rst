=====================
NuttX Protected Build
=====================

.. warning::
    Migrated from : 
    https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Protected+Build

The Traditional "Flat" Build
============================

The traditional NuttX build is a "flat" build. By flat, I mean that when 
you build NuttX, you end up with a single "blob" called ``nuttx``. All of the 
components of the build reside in the same address space. All components 
of the build can access all other components of the build.

The "Two Pass" Protected Build
==============================

The NuttX protected build, on the other hand, is a "two-pass" build and 
generates two "blobs": (1) a separately compiled and linked `kernel` blob 
called, again, `nuttx` and separately compiled and linked `user` blob called 
in ``nuttx_user.elf`` (in the existing build configurations). The user blob 
is created on pass 1 and the kernel blob is created on pass2.

These two make commands are identical:

.. code-block:: bash

    make
    make pass1 pass2

But the second is clearer and I prefer to use it for the protected build. 
In the second case, the user and kernel blobs are built separately; in the 
first, the kernel and user blob builds may be intermixed and somewhat 
confusing. You can also build the kernel and user blobs separately with 
one of the following commands:

.. code-block:: bash

    make pass1
    make pass2

At the end of the build, there will be several files in the top-level NuttX build directory. From Pass 1:

* ``nuttx_user.elf``. The pass1 user-space ELF file
* ``nuttx_user.hex``. The pass1 Intel HEX format file (selected in ``defconfig``)
* ``User.map``. Symbols in the user-space ELF file

From Pass 2:

* ``nuttx``. The pass2 kernel-space ELF file
* ``nuttx.hex``. The pass2 Intel HEX file (selected in ``defconfig``)
* ``System.map``. Symbols in the kernel-space ELF file

The Memory Protection Unit
==========================

If the MCU supports a Memory Protection Unit (MPU), then the logic within 
the kernel blob all execute in kernel-mode, i.e., with all privileges. 
These privileged threads can access all memory, all CPU instructions, 
and all MCU registers. The logic executing within the user-mode blob, 
on the other hand, all execute in user-mode with certain restrictions 
as enforced by the MCU and by the MPU. The MCU may restrict access to 
certain registers and machine instructions; with the MPU, access to all 
kernel memory resources are prohibited from the user logic. This includes 
the kernel blob's FLASH, .bss/.data storage, and the kernel heap memory.

Advantages of the Protected Build
=================================

The advantages of such a protected build are (1) security and (2) 
modularity. Since the kernel resources are protected, it will be much 
less likely that a misbehaving task will crash the system or that a 
wild pointer access will corrupt critical memory. This security also 
provides a safer environment in which to execute 3rd party software 
and prevents "snooping" into the kernel memory from the hosted applications.

Modularity is assured because there is a strict control of the exposed 
kernel interfaces. In the flat build, all symbols are exposed and there 
is no enforcement of a kernel API. With the protected build, on the 
other hand, all interactions with the kernel from the user application 
logic must use `system calls` (or `syscalls`) to interface with the OS. A 
system call is necessary to transition from user-mode to kernel-mode; 
all user-space operating system interfaces are via syscall `proxies`. 
Then, while in kernel mode, the kernel system call handler will 
perform the OS service requested by the application. At the 
conclusion of system processing, user-privileges are restored 
and control is return to the user application. Since the only 
interactions with the kernel can be through support system calls, 
modularity of the OS is guaranteed.

User-Space Proxies/Kernel-Space Stubs
=====================================

The same OS interfaces are exposed to the application in both the "flat" 
build and the protected build. The difference is that in the protected 
build, the user-code interfaces with a `proxy` for the OS function. For 
example, here is what a proxy for the OS ``getpid()`` interface:

.. code-block:: c

    #include <unistd.h>
    #include <syscall.h>
    pid_t getpid(void)
    {
        return (pid_t)sys_call0(SYS_getpid);
    }

Thus the ``getpid()`` proxy is a stand-in for the real OS ``getpid()`` interface 
that executes a system call so the kernel code can perform the real 
``getpid()`` operation on behalf of the user application. Proxies are 
auto-generated for all exported OS interfaces using the CSV file 
``syscall/syscall.csv`` and the program ``tools/mksyscalls``. Similarly, 
on the kernel-side, there are auto-generated `stubs` that map the 
system calls back into real OS calls. These, however, are internal 
to the OS and the implementation may be architecture-specific. 
See the ``README.txt`` files in those directories for further information.

Combining Intel HEX Files
=========================

One issue that you may face is that the two pass builds creates two 
FLASH images. Some debuggers that I use will allow me to write each 
image to FLASH separately. Others will expect to have a single Intel 
HEX image. In this latter case, you may need to combine the two Intel 
HEX files into one. Here is how you can do that:

1) The `tail` of the ``nuttx.hex`` file should look something like this 
   (with my comments and spaces added):

.. code-block:: bash

    $ tail nuttx.hex
    # 00, data records
    ...
    :10 9DC0 00 01000000000800006400020100001F0004
    :10 9DD0 00 3B005A0078009700B500D400F300110151
    :08 9DE0 00 30014E016D0100008D
    # 05, Start Linear Address Record
    :04 0000 05 0800 0419 D2
    # 01, End Of File record
    :00 0000 01 FF

Use an editor such as vi to remove the 05 and 01 records.

2) The `head` of the ``nuttx_user.hex`` file should look something like this 
   (again with my comments and spaces added):

.. code-block:: bash 

    $ head nuttx_user.hex
    # 04, Extended Linear Address Record
    :02 0000 04 0801 F1
    # 00, data records
    :10 8000 00 BD89 01084C800108C8110208D01102087E
    :10 8010 00 0010 00201C1000201C1000203C16002026
    :10 8020 00 4D80 01085D80010869800108ED83010829
    ...

Nothing needs to be done here. The ``nuttx_user.hex`` file should be fine.

3) Combine the edited nuttx.hex and un-edited ``nuttx_user.hex`` file to produce 
   a single combined hex file:

.. code-block:: bash

    $ cat nuttx.hex nuttx_user.hex >combined.hex

Then use the ``combined.hex`` file with for FLASH/JTAG tool. If you do this 
a lot, you will probably want to invest a little time to develop a tool 
to automate these steps.

Files and Directories
=====================

Here is a summary of directories and files used by the STM32F4Discovery 
protected build:

* ``boards/arm/stm32/stm32f4discovery/configs/kostest``. This is the kernel 
  mode OS test configuration. The two standard configuration files 
  can be found in this directory: (1) ``defconfig`` and (2) ``Make.defs``.
* ``boards/arm/stm32/stm32f4discovery/kernel``. This is the first past 
  build directory. The Makefile in this directory is invoked to 
  produce the pass1 object (``nuttx_user.elf`` in this case). The 
  second pass object is created by ``arch/arm/src/Makefile``. Also 
  in this directory is the file ``userspace.c``. The user-mode blob 
  contains a header that includes information need by the kernel 
  blob in order to interface with the user-code. That header is 
  defined in by this file.
* ``boards/arm/stm32/stm32f4discovery/scripts``. Linker scripts for 
  the kernel mode build are found in this directory. This includes 
  (1) ``memory.ld`` which hold the common memory map, (2) ``user-space.ld`` 
  that is used for linking the pass1 user-mode blob, and (3) 
  ``kernel-space.ld`` that is used for linking the pass1 kernel-mode blob.

Alignment, Regions, and Subregions
==================================

There are some important comments in the ``memory.ld`` 
file that are worth duplicating here:

"The STM32F407VG has 1024Kb of FLASH beginning at address 
0x0800:0000 and 192Kb of SRAM. SRAM is split up into three blocks:

* "112KB of SRAM beginning at address 0x2000:0000
* "16KB of SRAM beginning at address 0x2001:c000
* "64KB of CCM SRAM beginning at address 0x1000:0000

"When booting from FLASH, FLASH memory is aliased to address 
0x0000:0000 where the code expects to begin execution by jumping 
to the entry point in the 0x0800:0000 address range.

"For MPU support, the kernel-mode NuttX section is assumed to 
be 128Kb of FLASH and 4Kb of SRAM. That is an excessive amount 
for the kernel which should fit into 64KB and, of course, can 
be optimized as needed... Allowing the additional memory does 
permit addition debug instrumentation to be added to the kernel 
space without overflowing the partition.

"Alignment of the user space FLASH partition is also a critical 
factor: The user space FLASH partition will be spanned with a 
single region of size 2||n bytes. The alignment of the user-space 
region must be the same. As a consequence, as the user-space 
increases in size, the alignment requirement also increases.

"This alignment requirement means that the largest user space 
FLASH region you can have will be 512KB at it would have to be 
positioned at 0x08800000. If you change this address, don't 
forget to change the ``CONFIG_NUTTX_USERSPACE`` configuration 
setting to match and to modify the check in ``kernel/userspace.c``.

"For the same reasons, the maximum size of the SRAM mapping is 
limited to 4KB. Both of these alignment limitations could be 
reduced by using multiple MPU regions to map the FLASH/SDRAM 
range or perhaps with some clever use of subregions."

Memory Management
=================

At present, there are two options for memory management in the 
NuttX protected build:

Single User Heap
----------------

By default, there is only a single user-space heap and heap 
allocator that is shared by both kernel- and user-modes. 
PROs: Simple and makes good use of the heap memory space, 
CONs: Awkward architecture and no security for kernel-mode 
allocations.

Dual, Partitioned Heaps
-----------------------

Two configuration options can change this behavior:

* ``CONFIG_MM_MULTIHEAP=y``. This changes internal memory manager interfaces 
  so that multiple heaps can be supported.
* ``CONFIG_MM_KERNEL_HEAP=y``. Uses the multi-heap capability to enable 
  a kernel heap

If this both options are defined defined, the two heap partitions and 
two copies of the memory allocators are built:

One un-protected heap partition that will allocate user accessible memory 
that is shared by both the kernel- and user-space code. That allocator 
physically resides in the user address space so that it can be called 
directly by both the user- and kernel-space code. There is a header at 
the beginning of the user-space blob; the kernel-space code gets 
address of the user-space allocator from this header.

And another protected heap partition that will allocate protected 
memory that is only accessible from the kernel code. This allocator 
is built into the kernel block. This separate protected heap is 
required if you want to support security features.

NOTE: There are security issues with calling into the user space 
allocators in kernel mode. That is a security hole that could be 
exploit to gain control of the system! Instead, the kernel code 
should switch to user mode before entering the memory allocator 
stubs (perhaps via a trap). The memory allocator stubs should 
then trap to return to kernel mode (as does the signal handler now).

The Traditional Approach
------------------------

A more traditional approach would use something like the interface 
``sbrk()``. The ``sbrk()`` function adds memory to the heap space 
allocation of the calling process. In this case, there would 
still be kernel- and user-mode instances of the memory allocators. 
Each would ``sbrk()`` as necessary to extend their heap; the pages 
allocated for the kernel-mode allocator would be protected but 
the pages allocated for the user-mode allocator would not. 
PROs: Meets all of the needs. CONs: Complex. Memory losses 
due to quantization.

This approach works well with CPUs that have very capable 
Memory Management Units (MMUs) that can coalesce the 
srbk-ed chunks to a contiguous, `virtual` heap region. 
Without an MMU, the sbrk-ed memory would not be 
contiguous; this would limit the sizes of allocations 
due to the physical pages.

Many MCUs will have Memory Protection Units (MPUs) that can 
support the security features (only). However these lower 
end MPUs may not support sufficient mapping capability to 
support this traditional approach. The ARMv7-M MPU, for 
example, only supports eight protection regions to manage 
all FLASH and SRAM and so this approach would not be 
technically feasible for th ARMv7-M family (Cortex-M3/4).

Comparing the "Flat" Build Configuration with the Protected Build Configuration
===============================================================================

Compare, for example the configuration 
``boards/arm/stm32/stm32f4discovery/configs/ostest`` and the 
configuration ``boards/arm/stm32/stm32f4discovery/configs/kostest``. 
These two configurations are identical except that one builds a 
"flat" version of OS test and the other builds a kernel version 
of the OS test. See the file ``boards/arm/stm32/stm32f4discovery/README.txt`` 
for more details about those configurations.

The configurations can be compared using the ``cmpconfig`` tool:

.. code-block:: bash

    cd tools
    make -f Makefile.host cmpconfig
    cd ..
    tools/cmpconfig boards/arm/stm32/stm32f4discovery/configs/ostest/defconfig boards/arm/stm32/stm32f4discovery/configs/kostest/defconfig

Here is a summary of the meaning of all of the important differences in the 
configurations. This should be enough information for you to convert any 
configuration from a "flat" to a protected build:

* ``CONFIG_BUILD_2PASS=y``. This enables the two pass build.
* ``CONFIG_BUILD_PROTECTED=y``. This option enables the "two pass" 
  protected build.
* ``CONFIG_PASS1_BUILDIR="boards/arm/stm32/stm32f4discovery/kernel"``. 
  This tells the build system the (relative) location of the pass1 build directory.
* ``CONFIG_PASS1_OBJECT=""``. In some "two pass" build configurations, 
  the build system need to know the name of the first pass object. 
  This setting is not used for the protected build.
* ``CONFIG_NUTTX_USERSPACE=0x08020000``. This is the expected location 
  where the user-mode blob will be located. The user-mode blob 
  contains a header that includes information need by the kernel 
  blob in order to interface with the user-code. That header will 
  be expected to reside at this location.
* ``CONFIG_PASS1_TARGET="all"``. This is the build target to use for 
  invoking the pass1 make.
* ``CONFIG_MM_MULTIHEAP=y``. This changes internal memory manager 
  interfaces so that multiple heaps can be supported.
* ``CONFIG_MM_KERNEL_HEAP=y``. NuttX supports the option of using a 
  single user-accessible heap or, if this options is defined, 
  two heaps: (1) one that will allocate user accessible memory 
  that is shared by both the kernel- and user-space code, and 
  (2) one that will allocate protected memory that is only 
  accessible from the kernel code. Separate heap memory is required 
  if you want to support security features.
* ``CONFIG_MM_KERNEL_HEAPSIZE=8192``. This determines an approximate 
  size for the kernel heap. The standard heap space is partitioned 
  into a kernel- and user-heap space. This size of the kernel heap 
  is only approximate because the user heap is subject to stringent 
  alignment requirements. Because of the alignment requirements, the 
  actual size of the kernel heap could be considerable larger than this.
* ``CONFIG_BOARD_EARLY_INITIALIZE=y``. This setting enables a special, 
  `early` initialization call to initialize board-specific resources.
* ``CONFIG_BOARD_LATE_INITIALIZE=y``. This setting enables a special 
  initialization call to initialize `late` board-specific resources. 
  The difference between ``CONFIG_BOARD_EARLY_INITIALIZE`` and 
  ``CONFIG_BOARD_LATE_INITIALIZE`` is that the ``CONFIG_BOARD_EARLY_INITIALIZE`` 
  logic runs earlier in initialization before the full operating 
  system is up and running. ``CONFIG_BOARD_LATE_INITIALIZE``, on the 
  other hand, runs at the completion of initialization, just before 
  the user applications are started. Neither ``CONFIG_BOARD_EARLY_INITIALIZE`` 
  nor ``CONFIG_BOARD_LATE_INITIALIZE`` are used in the OS test 
  configuration but other configurations (such as NSH) 
  require some application-specific initialization before 
  the application can run. In the "flat" build, such initialization 
  is performed as part of the application start-up sequence. 
  These includes such things as initializing device drivers. 
  These same initialization steps must be performed in kernel 
  mode for the protected build and ``CONFIG_BOARD_LATE_INITIALIZE``. 
  See ``boards/arm/stm32/stm32f4discovery/src/up_boot.c`` for an 
  example of such board initialization code.
* ``CONFIG_NSH_ARCHINITIALIZE`` is not defined. The setting 
  ``CONFIG_NSH_ARCHINITIALIZE`` does not apply to the OS test 
  configuration, however, this is noted here as an example 
  of initialization that cannot be performed in the protected build.

Architecture-Specific Options:

* ``CONFIG_SYS_RESERVED=8``. The user application logic 
  interfaces with the kernel blob using system calls. 
  The architecture-specific logic may need to reserved a 
  few system calls for its own internal use. The ARMv7-M 
  architectures all require 8 reserved system calls.
* ``CONFIG_SYS_NNEST=2``. System calls may be nested. The 
  system must retain information about each nested system 
  call and this setting is used to set aside resources for 
  nested system calls. In the current architecture, a maximum 
  nesting level of two is all that is needed.
* ``CONFIG_ARMV7M_MPU=y``. This settings enables support for 
  the ARMv7-M Memory Protection Unit (MPU). The MPU is used 
  to prohibit user-mode access to kernel resources.
* ``CONFIG_ARMV7M_MPU_NREGIONS=8``. The ARMv7-M MPU supports 8 
  protection regions.

Size Expansion
==============

The protected build will, or course, result in a FLASH image that is 
larger than that of the corresponding "flat" build. How much larger? 
I don't have the numbers in hand, but you can build 
``boards/arm/stm32/stm32f4discovery/configs/nsh`` and 
``boards/arm/stm32/stm32f4discovery/configs/kostest`` and compare 
the resulting binaries for yourself using the ``size`` command.

Increases in size are expected because:

* The syscall layer is included in the protected build but not the flat 
  build.
* The kernel-size _syscal_l stubs will cause all enabled OS code to be 
  drawn into the build. In the flat build, only those OS interfaces 
  actually called by the application will be included in the final objects.
* The dual memory allocators will increase size.
* Code duplication. Some code, such as the C library, will be 
  duplicated in both the kernel- and user-blobs, and
* Alignment. The alignments required by the MPU logic will leave 
  relatively large regions of FLASH (and perhaps RAM) is not usable.

Performance Issues
==================

The only performance differences using the protected build should 
result as a consequence of the `sycalls` used to interact with the 
OS vs. the direct C calls as used in the flat build. If your 
performance is highly dependent upon high rate OS calls, then 
this could be an issue for you. But, in the typical application, 
OS calls do not often figure into the critical performance paths.

The `syscalls` are, ultimately, software interrupts. If the platform 
does not support prioritized, nested interrupts then the `syscall` 
execution could also delay other hardware interrupt processing. 
However, `sycall` processing is negligible: they really just 
configure to return to in supervisor mode and vector to the 
`syscall` stub. They should be lightning fast and, for the typical 
real-time applications, should cause no issues.