README.txt
==========

This board configuration will use QEMU to emulate generic ARM64 v8-A series
hardware platform and provides support for these devices:

 - GICv2 and GICv3 interrupt controllers
 - ARM Generic Timer
 - PL011 UART controller

Contents
========
  - Getting Started
  - Status
  - Platform Features
  - Debugging with QEMU
  - FPU Support and Performance
  - SMP Support
  - References

Getting Started
===============

1. Compile Toolchain
  1.1 Host environment
     GNU/Linux: Ubuntu 18.04 or greater
  1.2 Download and Install
     $ wget https://developer.arm.com/-/media/Files/downloads/gnu/11.2-2022.02/binrel/gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar.xz
     $ xz -d gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar.xz
     $ tar xf gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf.tar

     Put gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf/bin/ to your host PATH environment variable, like:
     $ export PATH=$PATH:/opt/software/arm/linaro-toolchain/gcc-arm-11.2-2022.02-x86_64-aarch64-none-elf/bin
     check the toolchain:
     $ aarch64-none-elf-gcc -v

2. Install QEMU
   In Ubuntu 18.04(or greater), install qemu:
   $ sudo apt-get install qemu-system-arm qemu-efi-aarch64 qemu-utils
   And make sure install is properly:
   $ qemu-system-aarch64 --help

3. Configuring and running
  3.1 Single Core (GICv3)
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv8a:nsh
   $ make
   Running with qemu
   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

  3.1.1 Single Core with network (GICv3)
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv8a:netnsh
   $ make
   Running with qemu
   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
     -mon chardev=con,mode=readline -kernel ./nuttx

  3.2 SMP (GICv3)
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv8a:nsh_smp
   $ make
   Running with qemu
   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
      -machine virt,virtualization=on,gic-version=3 \
      -net none -chardev stdio,id=con,mux=on -serial chardev:con \
      -mon chardev=con,mode=readline -kernel ./nuttx

  3.2.1 SMP (GICv3)
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv8a:netnsh_smp
   $ make
   Running with qemu
   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic \
     -machine virt,virtualization=on,gic-version=3 \
     -chardev stdio,id=con,mux=on -serial chardev:con \
     -global virtio-mmio.force-legacy=false \
     -netdev user,id=u1,hostfwd=tcp:127.0.0.1:10023-10.0.2.15:23,hostfwd=tcp:127.0.0.1:15001-10.0.2.15:5001 \
     -device virtio-net-device,netdev=u1,bus=virtio-mmio-bus.0 \
     -mon chardev=con,mode=readline -kernel ./nuttx

  3.3 Single Core (GICv2)
   Configuring NuttX and compile:
   $ ./tools/configure.sh -l qemu-armv8a:nsh_gicv2
   $ make
   Running with qemu
   $ qemu-system-aarch64 -cpu cortex-a53 -nographic \
     -machine virt,virtualization=on,gic-version=2 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con \
     -mon chardev=con,mode=readline -kernel ./nuttx

   Note:
   1. Make sure the aarch64-none-elf toolchain install PATH has been added to environment variable
   2. To quit QEMU, type Ctrl + X
   3. Nuttx default core number is 4, and Changing CONFIG_SMP_NCPUS > 4 and setting qemu command
     option -smp will boot more core. For qemu, core limit is 32.

Status
======

2022-11-18:
1. Added support for GICv2.

2. Added board configuration for nsh_gicv2.

2022-10-13:
1. Renamed the board configuration name from qemu-a53 to qemu-v8a.

2. Added the configurations for Cortex-A57 and Cortex-A72.

2022-07-01:

1. It's very stranger to see that signal testing of ostest is PASSED at Physical Ubuntu PC
   rather than an Ubuntu at VMWare. For Physical Ubuntu PC, I have run the ostest
   for 10 times at least but never see the crash again, but it's almost crashed every time
   running the ostest at Virtual Ubuntu in VMWare
   I check the fail point. It's seem at signal routine to access another CPU's task context reg
   will get a NULL pointer, but I watch the task context with GDB, everything is OK.
   So maybe this is a SMP cache synchronize issue? But I have done cache synchronize
   operation at thread switch and how to explain why the crash not happening at
   Physical Ubuntu PC?
   So maybe this is a qemu issue at VMWare. I am planning to run
   the arm64 to real hardware platform like IMX8 and will check the issue again

2022-06-12:

1. SMP is support at QEMU. Add psci interface, armv8 cache operation(data cache)
 and smccc support. The system can run into nsh shell, SMP test is PASSED, but
 ostest crash at signal testing


2022-05-22:
  Arm64 support version for NuttX is Ready, These Features supported:
1.Cotex-a53 single core support: With the supporting of GICv3,
  Arch timer, PL101 UART, The system can run into nsh shell.
  Running ostest seem PASSED.

2.qemu-a53 board configuration support: qemu-a53 board can configuring
  and compiling, And running with qemu-system-aarch64
  at Ubuntu 18.04.
3.FPU support for armv8-a: FPU context switching in NEON/floating-point
  TRAP was supported.  FPU registers saving at vfork and independent
  FPU context for signal routine was considered but more testing
  needs to be do.

Platform Features
=================

The following hardware features are supported:
+--------------+------------+----------------------+
| Interface    | Controller | Driver/Component     |
+==============+============+======================+
| GIC          | on-chip    | interrupt controller |
+--------------+------------+----------------------+
| PL011 UART   | on-chip    | serial port          |
+--------------+------------+----------------------+
| ARM TIMER    | on-chip    | system clock         |
+--------------+------------+----------------------+

The kernel currently does not support other hardware features on this
qemu platform.


Debugging with QEMU
===================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, make sure the following change have
   applied to defconfig.

+CONFIG_DEBUG_SYMBOLS=y

2. Run QEMU(at shell terminal 1)

   Single Core
   $ qemu-system-aarch64 -cpu cortex-a53 -nographic -machine virt,virtualization=on,gic-version=3 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con -mon chardev=con,mode=readline \
     -kernel ./nuttx -S -s
   SMP
   $ qemu-system-aarch64 -cpu cortex-a53 -smp 4 -nographic -machine virt,virtualization=on,gic-version=3 \
     -net none -chardev stdio,id=con,mux=on -serial chardev:con -mon chardev=con,mode=readline \
     -kernel ./nuttx -S -s

3. Run gdb with TUI, connect to QEMU, load nuttx and continue (at shell terminal 2)

   $ aarch64-none-elf-gdb -tui --eval-command='target remote localhost:1234' nuttx
   (gdb) set debug aarch64
   (gdb) c
   Continuing.
   ^C
   Program received signal SIGINT, Interrupt.
   arch_cpu_idle () at common/arm64_cpu_idle.S:37
   (gdb)
   (gdb) where
   #0  arch_cpu_idle () at common/arm64_cpu_idle.S:37
   #1  0x00000000402823ec in nx_start () at init/nx_start.c:742
   #2  0x0000000040280148 in arm64_boot_primary_c_routine () at common/arm64_boot.c:184
   #3  0x00000000402a5bf8 in switch_el () at common/arm64_head.S:201
   (gdb)

   SMP Case
   Thread 1 received signal SIGINT, Interrupt.
    arch_cpu_idle () at common/arm64_cpu_idle.S:37
   (gdb) info threads
    Id   Target Id                  Frame
  * 1    Thread 1 (CPU#0 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
    2    Thread 2 (CPU#1 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
    3    Thread 3 (CPU#2 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
    4    Thread 4 (CPU#3 [halted ]) arch_cpu_idle () at common/arm64_cpu_idle.S:37
   (gdb)

   Note:
   1. it will make your debugging more easier in source level if you setting
      CONFIG_DEBUG_FULLOPT=n. but there is a risk of stack overflow when the
      option is disabled. Just enlarging your stack size will avoid the
      issue (eg. enlarging CONFIG_DEFAULT_TASK_STACKSIZE)
   2. TODO: ARMv8-A Supporting for tools/nuttx-gdbinit


FPU Support and Performance
===========================
  I was using FPU trap to handle FPU context switch. For threads accessing
the FPU (FPU instructions or registers), a trap will happen at this thread,
the FPU context will be saved/restore for the thread at the trap handler.
  It will improve performance for thread switch since it's not to save/restore
the FPU context (almost 512 bytes) at the thread switch anymore. But some issue
need to be considered:

1. Floating point argument passing issue
    In many cases, the FPU trap is triggered by va_start() that copies
the content of FP registers used for floating point argument passing
into the va_list object in case there were actual float arguments from
the caller.
    adding -mgeneral-regs-only option will make compiler not use the FPU
register, we can use the following patch to syslog:

diff --git a/libs/libc/syslog/Make.defs b/libs/libc/syslog/Make.defs
index c58fb45512..acac6febaa
--- a/libs/libc/syslog/Make.defs
+++ b/libs/libc/syslog/Make.defs
@@ -26,3 +26,4 @@ CSRCS += lib_syslog.c lib_setlogmask.c

 DEPPATH += --dep-path syslog
 VPATH += :syslog
+syslog/lib_syslog.c_CFLAGS += -mgeneral-regs-only
   I cannot commit the patch for NuttX mainline because it's very special case
since ostest is using syslog for lots of information printing. but this is
a clue for FPU performance analysis. va_list object is using for many C code to
handle argument passing, but if it's not passing floating point argument indeed.
Add the option to your code maybe increase FPU performance

2. memset/memcpy issue
    For improve performance, the memset/memcpy implement for libc will
use the neon/fpu instruction/register. The FPU trap is also triggered
in this case.

we can trace this issue with Procfs:

nsh> cat /proc/arm64fpu
CPU0: save: 7 restore: 8 switch: 62 exedepth: 0
nsh>

after ostest
nsh> cat /proc/arm64fpu
CPU0: save: 1329 restore: 2262 switch: 4613 exedepth: 0
nsh>

Note:
save:    the counts of save for task FPU context
restore: the counts of restore for task FPU context
switch:  the counts of task switch

2. FPU trap at IRQ handler
    it's probably need to handle FPU trap at IRQ routine. Exception_depth is
handling for this case, it will inc/dec at enter/leave exception. If the
exception_depth > 1, that means an exception occurring when another exception
is executing, the present implement is to switch FPU context to idle thread,
it will handle most case for calling printf-like routine at IRQ routine.
    But in fact, this case will make uncertainty interrupt processing time sine
it's uncertainty for trap exception handling. It would be best to add
"-mgeneral-regs-only" option to compile the IRQ code avoiding accessing FP
register.
    if it's necessarily for the exception routine to use FPU, calling function to
save/restore FPU context directly maybe become a solution. Linux kernel introduce
kernel_neon_begin/kernel_neon_end function for this case. Similar function will
be add to NuttX if this issue need to be handle.

3. More reading 
for Linux kernel, please reference:
- https://www.kernel.org/doc/html/latest/arm/kernel_mode_neon.html

SMP Support
===========
1. Booting
   Primary core call sequence
     arm64_start
       ->arm64_boot_primary_c_routine
         ->arm64_chip_boot
           ->set init TBBR and Enable MMU
         ->nx_start
           ->OS component initialize
             ->Initialize GIC: GICD and Primary core GICR
           ->nx_smp_start
             for every CPU core
             ->up_cpu_start
               ->arm64_start_cpu(call PCSI to boot CPU)
               ->waiting for every core to boot
           ->nx_bringup

   Secondary Core call sequence
     arm64_start
       ->arm64_boot_secondary_c_routine
         ->Enable MMU
         ->Initialize GIC: Secondary core GICR
         ->Notify Primary core booting is Ready
         ->nx_idle_trampoline

2. interrupt

SGI
   SGI_CPU_PAUSE: for core pause request, for every core

PPI
   ARM_ARCH_TIMER_IRQ: timer interrupt, handle by primary Core

SPI
   CONFIG_QEMU_UART_IRQ: serial driver interrupt, handle by primary Core

3. Timer
 The origin design for ARMv8-A timer is assigned private timer to
 every PE(CPU core), the ARM_ARCH_TIMER_IRQ is a PPI so it's
 should be enabled at every core.

 But for NuttX, it's design only for primary core to handle timer
 interrupt and call nxsched_process_timer at timer tick mode.
 So we need only enable timer for primary core

 IMX6 use GPT which is a SPI rather than generic timer to handle
 timer interrupt

References
===========

1. (ID050815) ARM® Cortex®-A Series - Programmer’s Guide for ARMv8-A
2. (ID020222) Arm® Architecture Reference Manual - for A profile architecture
3. (ARM062-948681440-3280) Armv8-A Instruction Set Architecture
4. AArch64 Exception and Interrupt Handling
5. AArch64 Programmer's Guides Generic Timer
6. Arm Generic Interrupt Controller v3 and v4 Overview
7. Arm® Generic Interrupt Controller Architecture Specification GIC architecture version 3 and version 4
8. (DEN0022D.b) Arm Power State Coordination Interface Platform Design Document
