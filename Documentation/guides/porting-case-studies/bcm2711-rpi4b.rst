Porting to the BCM2711 (Raspberry Pi 4B)
========================================

This port was completed for the 12.7.0 version of the NuttX kernel, and was contributed by Matteo Golin.

The pull request with this initial support can be found at `apache/nuttx/pull/15188
<https://github.com/apache/nuttx/pull/15188>`_.

The port required support to be written for a new chip (the BCM2711) and a new board. Matteo created journal entries
while working on the initial port, which can be found `on his blog
<https://linguini1.github.io/blog/2024/12/25/nuttx-bcm2711.html>`_. The details below are a more concise summary of the
porting process.

Researching
-----------

The first step to porting a board to NuttX was researching the board and how NuttX works.

The BCM2711 is a quad-core ARM Cortex A72 based SoC, and it supports both aarch64 and 32 bit ARM architectures. I
focused on the aarch64 implementation only in this port. My first step was determining other boards already in the NuttX
kernel that used the aarch64 architecture, because that gives me a starting point to porting this new chip and board.

I primarily used the blog posts written by Lup Yuen Lee about porting NuttX to the PinePhone, another ARM Cortex-A based
device. The articles are listed `here <https://github.com/lupyuen/pinephone-nuttx>`_. Lup's articles provided me with an
understanding of the NuttX boot process, as well as which files from the aarch64 support on NuttX were pulled into the
build process for booting. He also showed how he created an initial UART driver using the NuttX structure for UART
drivers, which allowed him to get NSH appearing in the console.

Finally, I also of course needed the BCM2711 datasheet in order to figure out which registers were available to me for
creating peripheral drivers. The BCM2711 datasheet isn't exceptionally detailed on many of the features on the SoC, but
it did provide enough detail to set up interrupts and get UART working.

Adding to the source tree
-------------------------

In order to build my code with the NuttX build system, I would have to add the board and the BCM2711 chip to the source
tree for NuttX. This way, it would appear as an available configuration via the ``tools/configure.sh`` script and I
could select options for it with ``make menuconfig``.

The first thing to do was to add the chip, which goes under the ``arch/arm64`` directory because it is an ARM 64 bit
SoC. The chip directory must be added in two places: ``arch/arm64/include/bcm2711`` and ``arch/arm64/src/bcm2711``. C
files go in the ``src`` directory with some header files, and some specific header files go in the ``include``
directory.

In addition, in order to make the BCM2711 visible as a supported chip, I had to add it as an option in
``arch/arm64/Kconfig``. In order to do this, I just copy-pasted the entry for the Allwinner A64, since the two chips
were very similar. I had to change a few fields (for instance, selecting ``ARCH_CORTEX_A72`` instead of
``ARCH_CORTEX_A53``), but this was relatively simple to complete with the information about the SoC. I also needed to
specify ``ARMV8A_HAVE_GICv2``, since that is the interrupt controller used by the BCM2711. ``ARCH_HAVE_MULTICPU``
because it is a quad-core, and ``ARCH_USE_MMU`` because it has a memory management unit.

I also needed to now add the Raspberry Pi 4B board to the source tree. To do this, I copied the board folder for the
PinePhone (``boards/arm64/a64/pinephone``) and renamed it ``raspberrypi-4b``. I also deleted many of the files in this
folder since they weren't applicable to the Pi 4B, and substituted all mentions of the PinePhone with the Raspberry Pi
4B (in path names and header include guards).

I then added the Pi 4B to the list of supported boards in ``boards/Kconfig``. For this, I just needed to create an entry
with the name ``ARCH_BOARD_RASPBERRYPI_4B`` and write that it depends on the ``ARCH_CHIP_BCM2711``. No additional
options necessary! In two other places in this file I also had to add some directives to make sure the Kconfig for the
board was found properly. These set ``ARCH_BOARD`` to the name of the board directory "raspberrypi-4b" when the Pi 4B was
selected, and ``source``'d the Kconfig under ``boards/arm64/bcm2711/raspberrypi-4b`` when selected.

The default configuration for this board was copied from the PinePhone's NSH configuration, which I modified to use the
correct board name, chip, and hardware specific settings. It was still incomplete because there was no code to actually
boot into NSH, but it was a starting point.

This was basically all I needed for the board to show up as a possible configuration in the source tree!

Mapping out the chip
--------------------

To start writing code for the BCM2711, I needed to map out the chip. This included the register addresses and the memory
mapping, which could all be found in the BCM2711 datasheet. From looking at other implementations, the register
addresses are usually defined as C macros and kept in header files under ``arch/<architecture>/src/<chip>/hardware``.
This is where I put them as well, defining all the register mappings the different groups within individual files (i.e.
``bmc2711_i2c.h``, ``bcm2711_spi.h``, etc.).

Many peripherals had groupings of memory-mapped registers, defined using a base address and then offsets from that
address to access the different fields. For instance, the two mini-SPI peripherals had the same structure, each with 12
registers. The way I commonly saw these macros implemented was something like:

.. code-block:: c

   #define BCM_AUX_SPI1_BASEADDR (BCM_AUX_BASEADDR + BCM_AUX_SPI1_OFFSET)

   #define BCM_AUX_SPI_CNTL0_REG_OFFSET (0x00) /* SPI control register 0 */
   /* ... more register offsets */

   /* This allows you to choose which SPI interface base address to get the register for. */

   #define BCM_AUX_SPI_CNTL0(base) ((base) + BCM_AUX_SPI_CNTL0_REG_OFFSET)

In addition to the registers themselves, I also included macros to mask certain fields within the registers or set
certain values. This makes the code less error prone later, because any mistakes made while copying the long list of
fields and registers from the datasheet can be changed in one place.

.. code-block:: c

   #define BCM_SPI_CNTL0_EN (1 << 11) /* Enable SPI interface */

In addition to the registers, I also had to map the interrupts. This was done in ``include/bcm2711/irq.h``. I copied the
IRQ numbers from the datasheet and listed them all as macros with names. I also had to define the number of IRQS, which
was 216 in this case. The ``MPID_TO_CORE(mpid)`` macro was copied from another arm64 implementation.

.. code-block:: c

   #define NR_IRQS 216
   #define MPID_TO_CORE(mpid) (((mpid) >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK)

   /* VideoCore interrupts */

   #define BCM_IRQ_VC_BASE 96
   #define BCM_IRQ_VC(n) (BCM_IRQ_VC_BASE + n)

   #define BCM_IRQ_VC_TIMER0 BCM_IRQ_VC(0)
   #define BCM_IRQ_VC_TIMER1 BCM_IRQ_VC(1)
   /* More interrupts ... */

Finally was to define the memory mapping within the ``include/bcm2711/chip.h`` file. I did so simply since I was only
testing on the 4GB version of the BCM2711. The RAM starts at address 0, and is roughly 4GB in size. 64 MB of that is
reserved for the memory-mapped I/O, so I had to be sure to remove that. I also defined the load address of the kernel in
memory for the chip.

.. code-block:: c

   #define CONFIG_RAMBANK1_ADDR (0x000000000)

   /* Both the 4GB and 8GB ram variants use all the size in RAMBANK1 */

   #if defined(CONFIG_RPI4B_RAM_4GB) || defined(CONFIG_RPI4B_RAM_8GB)
   #define CONFIG_RAMBANK1_SIZE GB(4) - MB(64)
   #endif /* defined(CONFIG_RPI4B_RAM_4GB) || defined(CONFIG_RPI4B_RAM_8GB) */

   /* Raspberry Pi 4B loads NuttX at this address */

   #define CONFIG_LOAD_BASE 0x480000

The same load address had to be specified in the linker script for the Raspberry Pi 4B kernel. This scripts tells the
compiler how to lay out the kernel code in memory and what addresses to use. I was able to copy it from the PinePhone
and just change the load address to ``0x480000``.

Figuring out the boot
---------------------

The first thing I wanted to do was determine how much work had already been done for aarch64 that would allow me to more
easily complete the port. In Lup's blogs, he tested out support for his core type (ARM Cortex-A53 on the PinePhone) by
booting the aarch64 instance of QEMU with NuttX using that core. I decided to take the same approach, and was able to
successfully boot on ARM Cortex-A72 using QEMU following his blog. This was a nice confirmation that the hardware I was
using was already supported in NuttX for booting the OS and getting NSH working with a PL011 UART interface.

I cannot stress enough that the reason porting to this chip was made so much easier was because I am standing on the
shoulders of giants. NuttX contributors had already set up the boot scripts written in assembly, timer configuration,
interrupt handling and drivers for a lot of the standard features in aarch64 architectures. I did not have to deal with
any of this because of them, and it really cut down on the amount of assembly I had to read and understand. I also
barely had to write any assembly outside of debugging the boot process a little (we'll get to that later). Not to
mention I had Lup's well-written articles to guide me.

In order to compile and boot the board, I had to add a definition for ``g_mmu_config``, which I was confused about and
left empty initially just to get past the compilation stage. I also defined the ``GICR_OFFSET`` and ``GICR_BASE`` macros
for the GICv2 interrupt controller by copying them from the Allwinner chip, which used the same controller. After
reading further in Lup's blog, I learned that the boot script has a ``PRINT`` macro which is called early in the boot
process, and requires an implementation of ``up_lowputc`` to print to the console. This would be the first thing I need
to implement. This compiled, but when I booted the Pi, nothing happened.

After quite a while of trying different things and looking at other implementations, I noticed that many people were
using register manipulation directly in the early print functions. I decided I would do the same, but instead of
printing (a more complex operation), I would turn one of the GPIO pins high. I was able to measure this with my
multimeter and confirm that the GPIO did get set, so I knew that the ``arm64_earlyprint_init`` function was getting
called. Something was wrong with my UART configuration.

I then tried directly manipulating registers to put the text "hi" in the UART FIFO. When I booted again, this printed,
but then was followed by some garbled output. It appeared that the the ``char *`` pointer passed to the print function
was getting garbled. After troubleshooting by printing characters directly by calling my ``arm64_lowputc`` in the
assembly boot script, I discovered that I could print a string from the C definition if I declared the string as static.
I also investigated the elf generated by building and confirmed the string was located in ``.rodata``. I was suspicious
that I was loading the kernel incorrectly into memory and some addresses were getting mixed up. Sure enough, I had
defined the load address in the linker script as ``0x80000`` instead of ``0x480000``. Fixing this allowed me to see the
boot messages properly!

I received this message in the console:

.. code-block:: console

   ----gic_validate_dist_version: No GIC version detect
   arm64_gic_initialize: no distributor detected, giving up ret=-19
   _assert: Current Version: NuttX  12.6.0-RC0 6791d4a1c4-dirty Aug  4 2024 00:38:21 arm64
   _assert: Assertion failed panic: at file: common/arm64_fatal.c:375 task: Idle_Task process: Kernel 0x481418

I had accidentally kept the GICv3 in my config files when copying things from other boards, and changed it to GICv2.
That resolved the issue and presented me with a new one:

.. code-block:: console

   MESS:00:00:06.144520:0:----_assert: Current Version: NuttX  12.6.0-RC0 f81fb7a076-dirty Aug  4 2024 16:16:30 arm64
   _assert: Assertion failed panic: at file: common/arm64_fatal.c:375 task: Idle_Task process: Kernel 0x4811e4

After enabling all of the debug output in the build options, this became:

.. code-block:: console

   arm64_oneshot_initialize: cycle_per_tick 54000
   arm64_fatal_error: reason = 0
   arm64_fatal_error: CurrentEL: MODE_EL1
   arm64_fatal_error: ESR_ELn: 0xbf000002
   arm64_fatal_error: FAR_ELn: 0x0
   arm64_fatal_error: ELR_ELn: 0x48a458
   print_ec_cause: SError interrupt

This looked like an unhandled interrupt, and after narrowing down which line was failing by adding log statements to the
kernel code, I discovered it was due to the spinlock code. An exception was being caused by the ``ldaxr`` instruction,
which the ARM documentation said could only be used once the MMU was enabled. I then enabled the MMU as well as its
debug information and was greeted with the lovely error:

.. code-block:: console

   MESS:00:00:06.174977:0:----arm64_mmu_init: xlat tables:
   arm64_mmu_init: base table(L1): 0x4cb000, 64 entries
   arm64_mmu_init: 0: 0x4c4000
   arm64_mmu_init: 1: 0x4c5000
   arm64_mmu_init: 2: 0x4c6000
   arm64_mmu_init: 3: 0x4c7000
   arm64_mmu_init: 4: 0x4c8000
   arm64_mmu_init: 5: 0x4c9000
   arm64_mmu_init: 6: 0x4ca000
   init_xlat_tables: mmap: virt 4227858432x phys 4227858432x size 67108864x
   set_pte_table_desc:   
   set_pte_table_desc: 0x4cb018: [Table] 0x4c4000
   init_xlat_tables: mmap: virt 0x phys 0x size 1006632960x
   set_pte_table_desc:   
   set_pte_table_desc: 0x4cb000: [Table] 0x4c5000
   init_xlat_tables: mmap: virt 4718592x phys 4718592x size 192512x
   split_pte_block_desc: Splitting existing PTE 0x4c5010(L2)
   set_pte_table_desc:     
   set_pte_table_desc: 0x4c5010: [Table] 0x4c6000
   init_xlat_tables: mmap: virt 4911104x phys 4911104x size 81920x
   init_xlat_tables: mmap: virt 4993024x phys 4993024x size 65536x
   enable_mmu_el1: MMU enabled with dcache
   nx_start: Entry
   up_allocate_heap: heap_start=0x0x4d3000, heap_size=0x47b2d000
   mm_initialize: Heap: name=Umem, start=0x4d3000 size=1202900992
   mm_addregion: [Umem] Region 1: base=0x4d32a8 size=1202900304
   arm64_fatal_error: reason = 0
   arm64_fatal_error: CurrentEL: MODE_EL1
   arm64_fatal_error: ESR_ELn: 0x96000045
   arm64_fatal_error: FAR_ELn: 0x47fffff8
   arm64_fatal_error: ELR_ELn: 0x489d28
   print_ec_cause: Data Abort taken without a change in Exception level
   _assert: Current Version: NuttX  12.6.0-RC0 96be557b64-dirty Aug  5 2024 14:56:42 arm64
   _assert: Assertion failed panic: at file: common/arm64_fatal.c:375 task: Idle_Task process: Kernel 0x481a34
   up_dump_register: stack = 0x4d2e10
   up_dump_register: x0:   0x13                x1:   0x4d32c0
   up_dump_register: x2:   0xfe215040          x3:   0xfe215040
   up_dump_register: x4:   0x0                 x5:   0x0
   up_dump_register: x6:   0x1                 x7:   0xdba53f65cc808a8
   up_dump_register: x8:   0xc4276feb17c016ba  x9:   0xecbcfeb328124450
   up_dump_register: x10:  0xb7989dd7d34a1280  x11:  0x5ebf5f572386fdee
   up_dump_register: x12:  0x6f7c07d067f6e38   x13:  0x3f7b5adaf798b4d5
   up_dump_register: x14:  0xf3dffbe2e4cff736  x15:  0xd76b1c050c964ea0
   up_dump_register: x16:  0x6d6fa9cfeeb0eff8  x17:  0x1a051d808a830286
   up_dump_register: x18:  0x3f7b5adaf798b4bf  x19:  0x4d3000
   up_dump_register: x20:  0x47fffff0          x21:  0x4d32d0
   up_dump_register: x22:  0x47b2cd30          x23:  0x4d32a8
   up_dump_register: x24:  0x4d32b0            x25:  0x4806f4
   up_dump_register: x26:  0x2f56f66b2df71556  x27:  0x74ee6bbfb5d438f4
   up_dump_register: x28:  0x7ef57ab47b85f74f  x29:  0x9a7fa1cb06923003
   up_dump_register: x30:  0x489cf8          
   up_dump_register: 
   up_dump_register: STATUS Registers:
   up_dump_register: SPSR:      0x600002c5        
   up_dump_register: ELR:       0x489d28          
   up_dump_register: SP_EL0:    0x4d3000          
   up_dump_register: SP_ELX:    0x4d2f40          
   up_dump_register: TPIDR_EL0: 0x0               
   up_dump_register: TPIDR_EL1: 0x0               
   up_dump_register: EXE_DEPTH: 0x1                 

Some more debugging allowed me to determine that the ``CONFIG_RAM_START`` and ``CONFIG_RAM_SIZE`` macros in the
defconfig for my nsh configuration were still set to the values from the PinePhone that I copied from. I set these to
the correct values for the Raspberry Pi 4B and got much further!

.. code-block:: console

   MESS:00:00:06.211786:0:----irq_attach: In irq_attach
   irq_attach: before spin_lock_irqsave
   spin_lock_irqsave: me: 0
   spin_lock_irqsave: before spin_lock
   spin_lock: about to enter loop
   spin_lock: loop over
   spin_lock_irqsave: after spin_lock
   irq_attach: after spin_lock_irqsave
   irq_attach: before spin_unlock_irqrestore
   irq_attach: after spin_unlock_irqrestore
   arm64_serialinit: arm64_serialinit not implemented
   group_setupidlefiles: ERROR: Failed to open stdin: -38
   _assert: Current Version: NuttX  12.6.0-RC0 be262c7ad3-dirty Aug  5 2024 17:16:27 arm64
   _assert: Assertion failed : at file: init/nx_start.c:728 task: Idle_Task process: Kernel 0x48162c
   up_dump_register: stack = 0x4c0170
   up_dump_register: x0:   0x4c0170            x1:   0x0
   up_dump_register: x2:   0x0                 x3:   0x0
   up_dump_register: x4:   0x0                 x5:   0x0
   up_dump_register: x6:   0x3                 x7:   0x0
   up_dump_register: x8:   0x4c7468            x9:   0x0
   up_dump_register: x10:  0x4c7000            x11:  0x4
   up_dump_register: x12:  0x4b8000            x13:  0x4b7000
   up_dump_register: x14:  0x1                 x15:  0xfffffff7
   up_dump_register: x16:  0x48a654            x17:  0x0
   up_dump_register: x18:  0x1                 x19:  0x0
   up_dump_register: x20:  0x4ac181            x21:  0x4bf430
   up_dump_register: x22:  0x0                 x23:  0x4c0170
   up_dump_register: x24:  0x4c0170            x25:  0x2d8
   up_dump_register: x26:  0x240               x27:  0x4b7000
   up_dump_register: x28:  0xfdc3ed41d6862df6  x29:  0xbf8e8f7280a0100
   up_dump_register: x30:  0x481bf8          
   up_dump_register: 
   up_dump_register: STATUS Registers:
   up_dump_register: SPSR:      0x20000245        
   up_dump_register: ELR:       0x480230          
   up_dump_register: SP_EL0:    0x4c7000          
   up_dump_register: SP_ELX:    0x4c6e90          
   up_dump_register: TPIDR_EL0: 0x4bf430          
   up_dump_register: TPIDR_EL1: 0x4bf430          
   up_dump_register: EXE_DEPTH: 0x0               
   dump_tasks:    PID GROUP PRI POLICY   TYPE    NPX STATE   EVENT      SIGMASK          STACKBASE  STACKSIZE      USED   FILLED    COMMAND
   dump_tasks:   ----   --- --- -------- ------- --- ------- ---------- ---------------- 0x4c4000      4096       144     3.5%    irq
   dump_task:       0     0   0 FIFO     Kthread - Running            0000000000000000 0x4c5010      8176      1200    14.6%    Idle_Task
   
   CTRL-A Z for help | 115200 8N1 | NOR | Minicom 2.9 | VT102 | Offline | ttyUSB0

We actually got into tasks now! It appears stdin failed to open because in my Mini-UART driver implementation I had the
``attach`` and ``ioctl`` functions return ``-ENOSYS``. Just changing this to 0 for success in the interim allowed us to
get even further, and I could see the beginnings of NSH spawning.

.. code-block:: console

   mm_initialize: Heap: name=Umem, start=0x4cc000 size=4222828544
   mm_addregion: [Umem] Region 1: base=0x4cc2a8 size=4222827856
   mm_malloc: Allocated 0x4cc2d0, size 144
   mm_malloc: Allocated 0x4cc360, size 80
   gic_validate_dist_version: GICv2 detected
   up_timer_initialize: up_timer_initialize: cp15 timer(s) running at 54.0MHz
   arm64_oneshot_initialize: oneshot_initialize
   mm_malloc: Allocated 0x4cc3b0, size 48
   arm64_oneshot_initialize: cycle_per_tick 54000
   uart_register: Registering /dev/console
   mm_malloc: Allocated 0x4cc3e0, size 80
   mm_malloc: Allocated 0x4cc430, size 80
   uart_register: Registering /dev/ttys0
   mm_malloc: Allocated 0x4cc480, size 80
   mm_malloc: Allocated 0x4cc4d0, size 80
   mm_malloc: Allocated 0x4cc520, size 80
   mm_malloc: Allocated 0x4cc570, size 32
   mm_malloc: Allocated 0x4cc590, size 64
   work_start_highpri: Starting high-priority kernel worker thread(s)
   mm_malloc: Allocated 0x4cc5d0, size 336
   mm_malloc: Allocated 0x4cc720, size 8208
   nxtask_activate: hpwork pid=1,TCB=0x4cc5d0
   nx_start_application: Starting init thread
   task_spawn: name=nsh_main entry=0x48b24c file_actions=0 attr=0x4cbfa0 argv=0x4cbf98
   mm_malloc: Allocated 0x4ce730, size 1536
   mm_malloc: Allocated 0x4ced30, size 64
   mm_malloc: Allocated 0x4ced70, size 32
   mm_malloc: Allocated 0x4ced90, size 8208
   nxtask_activate: nsh_main pid=2,TCB=0x4ce730
   lib_cxx_initialize: _sinit: 0x4ad000 _einit: 0x4ad000
   mm_malloc: Allocated 0x4d0da0, size 848
   mm_free: Freeing 0x4d0da0
   mm_free: Freeing 0x4ced70
   mm_free: Freeing 0x4ced30
   nxtask_exit: nsh_main pid=2,TCB=0x4ce730
   mm_free: Freeing 0x4ced90
   mm_free: Freeing 0x4ce730
   nx_start: CPU0: Beginning Idle Loop

It seemed like we were waiting on an interrupt which never occurred. This was weird, because my Mini-UART driver had an
interrupt implementation and appeared to be written just fine. This took hours of debugging, logging from interrupt
handlers and dumping register values, but eventually I determined that the BCM2711 datasheet actually had an error where
the TX and RX interrupt fields were swapped in the datasheet. A blog post online had mentioned this for the BCM2835, but
it appeared to be an issue on this chip as well. Now we were booting into NSH!

It was at this point that the port is considered a success, since I was able to boot into NSH and successfully run the
``ostest`` benchmark. I went on to write the start of a few more drivers, like the GPIO driver, but this completed the
requirements for an initial port and is most of what ended up being submitted in the initial pull request.
