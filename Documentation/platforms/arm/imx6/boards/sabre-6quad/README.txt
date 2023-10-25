README.txt
==========

This directory holds a port of NuttX to the NXP/Freescale Sabre board
featuring the iMX 6Quad CPU.

This is a minimal port, used primarily for verifying SMP operation.  More
recently, a port to the i.MX RT was added.  This port has gotten more
support since it is better aligned with usage in embedded systems.  The
i.MX6 and the i.MX6 share IOMUXing and some peripherals.  It ought to be
a simple matter to backport some of the common drivers from i.MXRT to i.MX6.

Contents
========

  - Status
  - Platform Features
  - Serial Console
  - LEDs and Buttons
  - Using U-Boot to Run NuttX
  - Debugging with the Segger J-Link
  - SMP
  - Configurations

Status
======

2016-02-28: The i.MX6Q port is just beginning. A few files have been
  populated with the port is a long way from being complete or even ready to
  begin any kind of testing.

2016-03-12: The i.MX6Q port is code complete including initial
  implementation of logic needed for CONFIG_SMP=y  .  There is no clock
  configuration logic.  This is probably not an issue if we are loaded into
  SDRAM by a bootloader (because we cannot change the clocking anyway in
  that case).

  There is a lot of testing that could be done but, unfortunately, I still
  have no i.MX6 hardware to test on.

  In additional to the unexpected issues, I do expect to run into some
  cache coherency issues when I get to testing an SMP configuration.

2016-03-28:  I now have a used MCIMX6Q-SDB which is similar to the target
  configuration described below except that it does not have the 10.1" LVDS
  display.  Next step:  Figure out how to run a copy of NuttX using U-Boot.

2016-03-31: Most all of the boot of the NSH configuration seems to be
  working.  It gets to NSH and NSH appears to run normally.  Non-interrupt
  driver serial output to the VCOM console is working (llsyslog).  However,
  there does not appear to be any interrupt activity:  No timer interrupts,
  no interrupt driver serial console output (syslog, printf).

2016-05-16:  I now get serial interrupts (but not timer interrupts).  This
  involves a few changes to GIC bit settings that I do not fully understand.
  With this change, the NSH serial console works:

    MX6Q SABRESD U-Boot > ABEFGHILMN

    NuttShell (NSH)
    nsh>

  But there are still no timer interrupts.  LEDs do not appear to be working.

2016-05-17:  Timer interrupts now work.  This turned out to be just a minor
  bit setting error in the timer configuration.  LEDs were not working simply
  because board_autoled_initialize() was not being called in the board startup
  logic.

  At this point, I would say that the basic NSH port is complete.

2016-05-18: Started looking at the SMP configuration.  Initially, I verfied
  that the NSH configuration works with CONFIG_SMP_NCPUS=1.  Not a very
  interesting case, but this does exercise a lot of the basic SMP logic.

  When more than one CPU is configured, then there are certain failures that
  appear to be stack corruption problem.  See the open issues below under
  SMP.

2016-05-22: In a simple NSH case, SMP does not seem to be working.  But there
  are known SMP open issues so I assume if the tasking were stressed more there
  would be additional failures.  See the open issues below under SMP.

  An smp configuration was added.  This is not quite the same as the
  configuration that I used for testing.  I enabled DEBUG output, ran with
  only 2 CPUS, and disabled the RAMLOG:

    +CONFIG_DEBUG_FEATURES=y
    +CONFIG_DEBUG_INFO=y
    +CONFIG_DEBUG_SCHED=y
    +CONFIG_DEBUG_SYMBOLS=y

    -CONFIG_DEBUG_FULLOPT=y
    +CONFIG_DEBUG_NOOPT=y

    -CONFIG_SMP_NCPUS=4
    +CONFIG_SMP_NCPUS=2

    -CONFIG_RAMLOG=y
    -CONFIG_RAMLOG_SYSLOG=y
    -CONFIG_RAMLOG_BUFSIZE=16384
    -CONFIG_RAMLOG_NONBLOCKING=y
    -CONFIG_RAMLOG_NPOLLWAITERS=4

  I would also disable debug output from CPU0 so that I could better see the
  debug output from CPU1.  In drivers/syslog/vsyslog.c:

    +if (up_cpu_index() == 0) return 17; // REMOVE ME

2016-11-26: With regard to SMP, the major issue is cache coherency.  I added
  some special build logic to move spinlock data into the separate, non-
  cached section.  That gives an improvement in performance but there are
  still hangs.  These, I have determined, are to other kinds of cache
  coherency problems.  Semaphores, message queues, etc.  basically all
  shared data must be made coherent.

  I also added some SCU controls that should enable cache consistency for SMP
  CPUs, but I don't think I have that working right yet.  See the SMP section
  below for more information.

2016-11-28:  SMP is unusable until the SCU cache coherency logic is fixed.
  I do not know how to do that now.

2016-12-01:  I committed a completely untested SPI driver.  This was taken
  directly from the i.MX1 and is most certainly not ready for use yet.

2016-12-07:  Just a note to remind myself.  The PL310 L2 cache has *not*
  yet been enabled.

2018-02-06:  Revisited SMP to see how much has been broken due to bit rot.
  Several fixes were needed mostly due to:  (1) The new version of
  this_task() that calls sched_lock() and sched_unlock(), and (2) to
  deferred setting g_cpu_irqlock().  That latter setting is now deferred
  until nxsched_resume_scheduler() runs.  These commits were made:

    commit 50ab5d638a37b539775d1e60085f182bf26be57f
      sched/task:  It is not appropriate for logic in nxtask_exit() to call
      the new version of this_task().  sched/irq:  Remove redundant fetch
      of CPU index; boards/sabre-6quad: update README.

    commit 0ba78530164814360eb09ed9805137b934c6f03b
      sched/irq: Fix a infinite recursion problem that a recent change
      introduced into the i.MX6 SMP implementation.

    commit 8aa15385060bf705bbca2c22a5682128740e55a8
      arch/arm/src/armv7-a:  Found some additional places were the new
      this_task() function cannot be called in the i.MX6 SMP configuration.

    commit de34b4523fc33c6f2f20619349af8fa081a3bfcd
      sched/ and arch/arm/src/armv7-a:  Replace a few more occurrences
      of this_task() with current_task(cpu) in an effort to get the i.MX6
      working in SMP mode again.  It does not yet work, sadly.

    commit cce21bef3292a40dcd97b6176ea016e2b559de8b
      sched/sched: sched_lock() and sched_unlock().. back out some changes
      I made recently.  The seemed correct but apparently not.  Also
      reorder to logic so that g_global_lockcount is incremented for the very
      minimum amount of time.

  With these changes, basic SMP functionality is restored and there are no
  known issues (Configuration 'smp' with 4 CPUs and data cache disabled).
  It is possible, however, that additional changes similar to the above will
  be required in other areas of the OS, but none such are known as of this
  writing.  Insufficient stress testing has been done to prove that the
  solution is stable.

2018-06-08:  Again revisited SMP.  There appears to be a memory corruption problem.
  This is rarely seen with the SMP test but you enable the OS test in the smp
  configuration, you will see a crash due to memory corruption consistently,
  specially in the nested signal test (apps/examples/ostest/signest.c).

2018-06-20:  There was a problem with the Interrupt Stack for SMP in
  arch/arm/src/armv7-a/arm_vectors.S:  There is only one interrupt stack for
  all CPUs!  A fix for this was put in place on 2018-06-21.  Big Improvement!
  But this does not completely eliminate instabilities which seem to be
  related to memory corruption -- mm_mallinfo() asserts.

Platform Features
=================

Processor:
  - i.MX 6Quad or 6DualLite 1 GHz ARM Cortex-A9 processor
Memory/storage:
  - 1 GB DDR3 SDRAM up to 533 MHz (1066 MTPS) memory
  - 8 GB eMMC flash
  - 4 MB SPI NOR flash
Display:
  - 10.1" 1024 x 768 LVDS display with integrated P-cap sensing
  - HDMI connector
  - LVDS connector (for optional second display)
  - LCD expansion connector (parallel, 24-bit)
  - EPDC expansion connector (for 6DualLite only)
  - MIPI DSI connector (two data lanes, 1 GHz each)
User Interface:
  - 10.1" capacitive multitouch display
  - Buttons: power, reset, volume
Power Management:
  - Proprietary PF0100 PMIC
Audio:
  - Audio codec
  - 2x digital microphones
  - 2x 3.5 mm audio ports
  - Dual 1 watt speakers
Expansion Connector:
  - Camera MIPI CSI port
  - I2C, SPI signals
Connectivity:
  - 2x full-size SD/MMC card slots
  - 7-pin SATA data connector
  - 10/100/1000 Ethernet port
  - 1x USB 2.0 OTG port (micro USB)
Debug:
  - JTAG connector (20-pin)
  - 1x Serial-to-USB connector (for JTAG)
OS Support:
  - Linux® and Android™ from NXP/Freescale
  - Others supported via third party (QNX, Windows Embedded)
Tools Support:
  - Manufacturing tool from NXP/Freescale
  - IOMUX tool from NXP/Freescale
  - Lauterbach, ARM (DS-5), IAR and Macraigor
Additional Features:
  - Proprietary 3-axis accelerometer
  - Proprietary 3D magnetometer
  - Ambient light sensor
  - GPS receiver module
  - 2x 5MP cameras
  - Battery charger
  - Battery connectors (battery not included)

Serial Console
==============

A DEBUG VCOM is available MICRO USB AB 5 J509.  This corresponds to UART1
from the i.MX6.  UART1 connects to J509 via the CSIO_DAT10 and CSIO_DAT11
pins

LEDs and Buttons
================

LEDs
----
A single LED is available driven GPIO1_IO02.  On the schematic this is
USR_DEF_RED_LED signal to pin T1 (GPIO_2).  This signal is shared with
KEY_ROW6 (ALT2).  A high value illuminates the LED.

This LED is not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/sam_autoleds.c. The LED is used to encode
OS-related events as follows:

  ------------------- ----------------------- ------
  SYMBOL              Meaning                 LED
  ------------------- ----------------------- ------
  LED_STARTED         NuttX has been started  OFF
  LED_HEAPALLOCATE    Heap has been allocated OFF
  LED_IRQSENABLED     Interrupts enabled      OFF
  LED_STACKCREATED    Idle stack created      ON
  LED_INIRQ           In an interrupt         N/C
  LED_SIGNAL          In a signal handler     N/C
  LED_ASSERTION       An assertion failed     N/C
  LED_PANIC           The system has crashed  FLASH

Thus if the LED is statically on, NuttX has successfully  booted and is,
apparently, running normally.  If the LED is flashing at approximately
2Hz, then a fatal error has been detected and the system has halted.

Buttons
-------

Using U-Boot to Run NuttX
=========================

The MCIMX6Q-SDB comes with a 8GB SD card containing the U-Boot and Android.
You simply put the SD card in the SD card slot SD3 (on the bottom of the
board next to the HDMI connect) and Android 4.2.2.1 will boot.

But we need some other way to boot NuttX.  Here are some things that I have
experimented with.

Building U-Boot (Failed Attempt #1)
-----------------------------------

I have been unsuccessful getting building a working version of u-boot from
scratch.  It builds, but it does not run.  Here are the things I did:

1. Get a copy of the u-boot i.MX6 code and Android GCC toolchain

    $ git clone https://source.codeaurora.org/external/imx/uboot-imx.git -b nxp/imx_v2009.08
    $ git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6

2. Build U-Boot for the i.MX6Q Sabre using the following steps.  This
   assumes that you have the path to the above toolchain at the
   beginning of your PATH variable:

    $ cd uboot-imx
    $ export ARCH=arm
    $ export CROSS_COMPILE=arm-eabi-
    $ make mx6q_sabresd_android_config
    $ make

  This should create a number of files, including u-boot.bin

3. Format an SD card

  Create a FAT16 partition at an offset of about 1MB into the SD card.
  This is where we will put nuttx.bin.

4. Put U-Boot on SD.

    $ dd if=u-boot.bin of=/dev/<your-sd-card> bs=1k
    $ sync

  Your SD card device is typically something in /dev/sd<X> or
  /dev/mmcblk<X>. Note that you need write permissions on the SD card
  for the command to succeed, so you might need to su - as root, or use
  sudo, or do a chmod a+w as root on the SD card device node to grant
  permissions to users.

Using the Other SD Card Slot (Failed Attempt #2)
------------------------------------------------

Another option is to use the version u-boot that came on the 8GB but put
NuttX on another SD card inserted in the other SD card slot at the opposite
corner of the board.

To make a long story short:  This doesn't work.  As far as I can tell,
U-Boot does not support any other other SC card except for mmc 2 with is the
boot SD card slot.

Replace Boot SD Card (Successful Attempt #3)
--------------------------------------------

What if you remove the SD card after U-boot has booted, then then insert
another SD card containing the nuttx.bin image?

1. Build nuttx.bin and copy it only a FAT formatted SD card.  Insert the SD
   card containing NuttX into the "other" SD card slot.  Insert the 8GB SD
   card with U-boot already on it in the normal, boot SD card slot.

2. Connect the VCOM port using the USB port next to the boot SD card slot.

3. Start a console at 11500 8N1 on the VCOM port

4. Power up the board with the 8GB SD card in place.  U-Boot will start and
   countdown before starting Linux.  Press enter to break into U-Boot before
   Linux is started.

5. Remove the 8GB U-Boot SD card; insert in its place.

6. Rescan the SD card:

  MX6Q SABRESD U-Boot > mmc dev 2
  mmc2 is current device
  MX6Q SABRESD U-Boot > mmc rescan
  MX6Q SABRESD U-Boot > fatls mmc 2
              system volume information/
      87260   nuttx.bin

  1 file(s), 1 dir(s)

7. Then we can boot NuttX off the rescanned SD card:

     MX6Q SABRESD U-Boot > fatload mmc 2 0x10800000 nuttx.bin
     reading nuttx.bin

     87260 bytes read
     MX6Q SABRESD U-Boot > go 0x10800040
     ## Starting application at 0x10800040 ...

   That seems to work okay.

Use the FAT Partition on the 8GB SD Card (Untested Idea #4)
-----------------------------------------------------------

Partition 4 on the SD card is an Android FAT file system.  So one thing you
could do would be put the nuttx.bin file on that partition, then boot like:

     MX6Q SABRESD U-Boot > fatload mmc 2:4 0x10800000 nuttx.bin

SD Card Image Copy (Successful Attempt #5)
------------------------------------------

You can use the 'dd' command to copy the first couple of megabytes from the
8GB SD card and copy that to another SD card.  You then have to use 'fdisk'
to fix the partition table and to add a single FAT16 partition at an offset
of 1MB or so.

1. Insert the 8GB boot SD card into your PC: Copy the first 2Mb from the SD
   card to a file:

     $ dd if=/dev/sdh of=sdh.img bs=512 count=4096

2. Remove the 8GB boot SD card and replace it with a fresh SD card.  Copy the
   saved file to the first the new SD card:

     $ dd of=/dev/sdh if=sdh.img bs=512 count=4096

3. Then use 'fdisk' to:

   - Remove all of the non-existent partitions created by the 'dd' copy.
   - Make a single FAT16 partition at the end of the SD card.

   You will also need to format the partion for FAT.

4. You can put nuttx.bin here and then boot very simply with:

     MX6Q SABRESD U-Boot > fatload mmc 2:1 0x10800000 nuttx.bin
     MX6Q SABRESD U-Boot > go 0x10800040

A little hokey, but not such a bad solution.

TFTPBOOT (Successful Attempt #6)
------------------------------------------

If you can prepare tftp server, this approach would be easy

1. Copy nuttx.bin to the tftp server (e.g. /var/lib/tftpboot/ )

2. Load nuttx.bin from the server and boot

     MX6Q SABRESD U-Boot > setenv ipaddr 192.168.10.103
     MX6Q SABRESD U-Boot > setenv serverip 192.168.10.16
     MX6Q SABRESD U-Boot > setenv image nuttx.bin
     MX6Q SABRESD U-Boot > tftp ${loadaddr} ${image}
     PHY indentify @ 0x1 = 0x004dd074
     FEC: Link is Up 796d
     Using FEC0 device
     TFTP from server 192.168.10.16; our IP address is 192.168.10.103
     Filename 'nuttx.bin'.
     Load address: 0x10800000
     Loading: ###############
     done
     Bytes transferred = 217856 (35300 hex)
     MX6Q SABRESD U-Boot > go ${loadaddr}
     ## Starting application at 0x10800000 ...

     NuttShell (NSH) NuttX-10.0.1
     nsh>

Debugging with the Segger J-Link
================================

These procedures work for debugging the boot-up sequence when there is a
single CPU running and not much else going on.  If you want to do higher
level debugger, you will need something more capable.  NXP/Freescale suggest
some other debuggers that you might want to consider.

These instructions all assume that you have built NuttX with debug symbols
enabled.  When debugging the nuttx.bin file on the SD card, it is also
assumed the nuttx ELF file with the debug symbol addresses is from the
same build so that the symbols match up.

Debugging the NuttX image on the SD card
----------------------------------------

1. Connect the J-Link to the 20-pin JTAG connector.

2. Connect the "USB TO UART" USB VCOM port to the host PC.  Start a
   terminal emulation program like TeraTerm on Minicom.  Select the USB
   VCOM serial port at 115200 8N1.

   When you apply power to the board, you should see the U-Boot messages in
   the terminal window.  Stop the U-Boot countdown to get to the U-Boot
   prompt.

3. Start the Segger GDB server:

     Target:           MCIMX6Q6
     Target Interface: JTAG

   If the GDB server starts correctly you should see the following in the
   Log output:

     Waiting for GDB Connection

4. In another Xterm terminal window, start arm-none-eabi-gdb and connect to
   the GDB server.

   From the Xterm Window:
     $ arm-none-eabi-gdb

   You will need to have the path to the arm-none-eabi-gdb program in your
   PATH variable.

   Then from GDB:
     gdb> target connect localhost:2331
     gdb> mon halt

5. Start U-boot under GDB control:

   From GDB:
     gdb> mon reset
     gdb> mon go

   Again stop the U-Boot countdown to get to the U-Boot prompt.

6. Load NuttX from the SD card into RAM

   From U-Boot:
     MX6Q SABRESD U-Boot > fatload mmc 2:1 0x10800000 nuttx.bin

7. Load symbols and set a breakpoint

   From GDB:
     gdb> mon halt
     gdb> file nuttx
     gdb> b __start
     gdb> c

   __start is the entry point into the NuttX binary at 0x10800040.  You can,
   of course, use a different symbol if you want to start debugging later
   in the boot sequence.

8. Start NuttX

   From U-Boot:
     MX6Q SABRESD U-Boot > go 0x10800040

9. You should hit the breakpoint that you set above and be off and
   debugging.

Debugging a Different NuttX Image
---------------------------------

Q: What if I want do run a different version of nuttx than the nuttx.bin
   file on the SD card.  I just want to build and debug without futzing with
   the SD card.  Can I do that?

A: Yes with the following modifications to the procedure above.

   - Follow steps 1-5, i.e.,

       1. Connect the J-Link to the 20-pin JTAG connector.
       2. Connect the "USB TO UART" USB VCOM port to the host PC and start a
          terminal emulation program.
       3. Start the Segger GDB server.
       4. Start arm-none-eabi-gdb and connect to the GDB server.
       5. Start U-boot under GDB control, stopping the countdown to get
          the U-boot prompt.

   - Skip step 6, don't bother to load NuttX into RAM
   - In step 7, load NuttX into RAM like this:

       gdb> mon halt
       gdb> load nuttx <-- Loads NuttX into RAM at 0x010800000
       gdb> file nuttx
       gdb> b __start
       gdb> c

   - Then after step 7, you should hit the breakpoint at the instruction you
     just loaded at address 0x10800040.

   - Or, in step 6, instead of continuing ('c') which will resume U-Boot,
     even just:

       gdb> mon halt
       gdb> load nuttx <-- Loads NuttX into RAM at 0x010800000
       gdb> file nuttx
       gdb> mon reg pc 0x10800040
       gdb> s

     The final single will then step into the freshly loaded program.
     You can then forget about steps 8 and 9.

     This is, in fact, my preferred way to debug.

     NOTE:  Setting the PC to 0x10800040 is a superstituous step.  The PC
     will be set 0x10800040 by the 'load nuttx' command.

   You can restart the debug session at any time at the gdb> prompt by:

       gdb> mon reset
       gdb> mon go

   That will restart U-Boot and you have to press ENTER in the terminal
   window to stop U-Boot.  Restarting U-Boot is a necessary part of the
   restart process because you need to put the hardware back in its initial
   state before running NuttX.

   Then this will restart the debug session just as before:

       gdb> mon halt
       gdb> load nuttx <-- Loads NuttX into RAM at 0x010800000
       gdb> file nuttx
       gdb> mon reg pc 0x10800040
       gdb> s

Debugging with QEMU
================================

The nuttx ELF image can be debugged with QEMU.

1. To debug the nuttx (ELF) with symbols, following change must
   be applied to defconfig.

diff --git a/boards/arm/imx6/sabre-6quad/configs/nsh/defconfig b/boards/arm/imx6/sabre-6quad/configs/nsh/defconfig
index b15becbb51..3ad4d13ad7 100644
--- a/boards/arm/imx6/sabre-6quad/configs/nsh/defconfig
+++ b/boards/arm/imx6/sabre-6quad/configs/nsh/defconfig
@@ -21,11 +21,12 @@ CONFIG_ARCH_STACKDUMP=y
 CONFIG_BOARD_LOOPSPERMSEC=99369
 CONFIG_BOOT_RUNFROMSDRAM=y
 CONFIG_BUILTIN=y
+CONFIG_DEBUG_FULLOPT=y
+CONFIG_DEBUG_SYMBOLS=y
 CONFIG_DEV_ZERO=y

2. Please note that QEMU does not report PL310 (L2CC) related
   registers correctly, so if you enable CONFIG_DEBUG_ASSERTION
   the nuttx will stop with DEBUGASSERT(). To avoid this,
   comment out the following configs in defconfig

-CONFIG_ARMV7A_ASSOCIATIVITY_16WAY=y
-CONFIG_ARMV7A_L2CC_PL310=y
-CONFIG_ARMV7A_WAYSIZE_64KB=y
-CONFIG_PL310_LOCKDOWN_BY_LINE=y
-CONFIG_PL310_LOCKDOWN_BY_MASTER=y

3. Run QEMU

   Run qemu with following options, these options do not load nuttx.
   Instead, just stops the emulated CPU like "reset halt" with OpenOCD.

   $ qemu-system-arm -M sabrelite -smp 4 -nographic -s -S

   NOTE: -smp 4 option should be used for both nsh configuration
   (non-SMP) and smp configuration (regardless of CONFIG_SMP_NCPUS)

   To quit QEMU, type Ctrl-A + X

3. Run gdb, connect to QEMU, load nuttx and continue

   $ arm-none-eabi-gdb ./nuttx
   (gdb) target  extended-remote :1234
   Remote debugging using :1234
   0x00000000 in ?? ()
   (gdb) load nuttx
   Loading section .text, size 0x17f6b lma 0x10800000
   Loading section .ARM.exidx, size 0x8 lma 0x10817f6c
   Loading section .data, size 0x98 lma 0x10817f74
   Start address 0x10800040, load size 98315
   Transfer rate: 8728 KB/sec, 1927 bytes/write.
   (gdb) c
   Continuing.
   ^C
   Thread 1 received signal SIGINT, Interrupt.
   up_idle () at common/up_idle.c:78
   78	}
   (gdb) where
   #0  up_idle () at common/up_idle.c:78
   #1  0x10801ba4 in nx_start () at init/nx_start.c:874
   #2  0x00000000 in ?? ()
   (gdb) info threads
     Id   Target Id         Frame
   * 1    Thread 1 (CPU#0 [halted ]) up_idle () at common/up_idle.c:78
     2    Thread 2 (CPU#1 [halted ]) 0x00000000 in ?? ()
     3    Thread 3 (CPU#2 [halted ]) 0x00000000 in ?? ()
     4    Thread 4 (CPU#3 [halted ]) 0x00000000 in ?? ()

SMP
===

The i.MX6 6Quad has 4 CPUs.  Support is included for testing an SMP
configuration.  That configuration is still not yet ready for usage but can
be enabled with the following configuration settings:

  RTOS Features -> Tasks and Scheduling
    CONFIG_SPINLOCK=y
    CONFIG_SMP=y
    CONFIG_SMP_NCPUS=4

Open Issues:

1. Currently all device interrupts are handled on CPU0 only.  Critical sections will
   attempt to disable interrupts but will now disable interrupts only on the current
   CPU (which may not be CPU0).  There is a spinlock to prohibit entrance into these
   critical sections in interrupt handlers of other CPUs.

   When the critical section is used to lock a resource that is also used by
   interrupt handling, the interrupt handling logic must also take the spinlock.
   This will cause the interrupt handlers on other CPUs to spin until
   leave_critical_section() is called.  More verification is needed.

2. Recent redesigns to SMP of another ARMv7-M platform have made changes to the OS
   SMP support.  There are no known problem but the changes have not been verified
   fully (see STATUS above for 2019-02-06).

Configurations
==============

Information Common to All Configurations
----------------------------------------
Each Sabre-6Quad configuration is maintained in a sub-directory and
can be selected as follow:

  tools/configure.sh sabre-6quad:<subdir>

Before building, make sure the PATH environment variable includes the
correct path to the directory than holds your toolchain binaries.

And then build NuttX by simply typing the following.  At the conclusion of
the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

  make oldconfig
  make

The <subdir> that is provided above as an argument to the tools/configure.sh
must be is one of the following.

NOTES:

  1. These configurations use the mconf-based configuration tool.  To
     change any of these configurations using that tool, you should:

     a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
        see additional README.txt files in the NuttX tools repository.

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART1 which is a available to the host PC from the USB
     micro AB as a VCOM part.

  3. All of these configurations are set up to build under Windows using the
     "GNU Tools for ARM Embedded Processors" that is maintained by ARM
     (unless stated otherwise in the description of the configuration).

       https://developer.arm.com/open-source/gnu-toolchain/gnu-rm

     That toolchain selection can easily be reconfigured using
     'make menuconfig'.  Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y               : Window environment
       CONFIG_WINDOWS_CYGWIN=y             : Cywin under Windows

     System Type -> Toolchain:
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y  : GNU ARM EABI toolchain

Configuration sub-directories
-----------------------------

  nsh
  ---
    This is a NuttShell (NSH) configuration that uses the NSH library
    at apps/nshlib with the start logic at apps/examples/nsh.

    NOTES:

    1. This configuration assumes that we are loaded into SDRAM and
       started via U-Boot.

    2. The serial console is configured by default for use UART1, the
       USB VCOM port (UART1), same as the serial port used by U-Boot.
       You will need to reconfigure if you want to use a different UART.

    3. NSH built-in applications are supported, but no built-in
       applications are enabled.

       Binary Formats:
         CONFIG_BUILTIN=y           : Enable support for built-in programs

       Application Configuration:
         CONFIG_NSH_BUILTIN_APPS=y  : Enable starting apps from NSH command line

    4. The RAMLOG is enabled.  All SYSLOG (DEBUG) output will go to the
       RAMLOG and will not be visible unless you use the nsh 'dmesg'
       command.  To disable this RAMLOG feature, disable the following:

       Device Drivers:  CONFIG_RAMLOG

  smp
  ---
    This is a configuration of testing the SMP configuration.  It is
    essentially equivalent to the nsh configuration except has SMP enabled
    and supports apps/testing/smp.

    Sample output of the SMP test is show below (Configuration all 4 CPUs
    but with data cache disabled):

      NuttShell (NSH) NuttX-7.23
      nsh> smp
        Main[0]: Running on CPU0
        Main[0]: Initializing barrier
      Thread[1]: Started
        Main[0]: Thread 1 created
      Thread[1]: Running on CPU0
        Main[0]: Now running on CPU1
      Thread[2]: Started
        Main[0]: Thread 2 created
      Thread[2]: Running on CPU1
        Main[0]: Now running on CPU2
      Thread[3]: Started
        Main[0]: Thread 3 created
      Thread[3]: Running on CPU2
        Main[0]: Now running on CPU3
      Thread[4]: Started
      Thread[4]: Running on CPU3
        Main[0]: Thread 4 created
        Main[0]: Now running on CPU0
      Thread[5]: Started
      Thread[5]: Running on CPU0
        Main[0]: Thread 5 created
      Thread[6]: Started
      Thread[6]: Running on CPU0
        Main[0]: Thread 6 created
      Thread[7]: Started
      Thread[7]: Running on CPU0
        Main[0]: Thread 7 created
      Thread[8]: Started
      Thread[8]: Running on CPU0
        Main[0]: Thread 8 created
      Thread[2]: Now running on CPU0
      Thread[3]: Now running on CPU0
      Thread[4]: Now running on CPU0
      Thread[3]: Now running on CPU2
      Thread[3]: Now running on CPU0
      Thread[5]: Now running on CPU1
      Thread[5]: Now running on CPU0
      Thread[6]: Calling pthread_barrier_wait()
      Thread[8]: Calling pthread_barrier_wait()
      Thread[3]: Calling pthread_barrier_wait()
      Thread[5]: Calling pthread_barrier_wait()
      Thread[1]: Calling pthread_barrier_wait()
      Thread[2]: Now running on CPU2
      Thread[2]: Calling pthread_barrier_wait()
      Thread[7]: Now running on CPU3
      Thread[4]: Now running on CPU1
      Thread[4]: Calling pthread_barrier_wait()
      Thread[7]: Calling pthread_barrier_wait()
      Thread[7]: Back with ret=PTHREAD_BARRIER_SERIAL_THREAD (I AM SPECIAL)
      Thread[6]: Back with ret=0 (I am not special)
      Thread[8]: Back with ret=0 (I am not special)
      Thread[3]: Back with ret=0 (I am not special)
      Thread[5]: Back with ret=0 (I am not special)
      Thread[1]: Back with ret=0 (I am not special)
      Thread[2]: Back with ret=0 (I am not special)
      Thread[4]: Back with ret=0 (I am not special)
      Thread[7]: Now running on CPU1
      Thread[6]: Now running on CPU2
      Thread[3]: Now running on CPU1
      Thread[5]: Now running on CPU2
      Thread[1]: Now running on CPU1
      Thread[4]: Now running on CPU3
      Thread[2]: Now running on CPU0
      Thread[7]: Now running on CPU0
      Thread[6]: Now running on CPU0
      Thread[3]: Now running on CPU0
      Thread[4]: Now running on CPU0
      Thread[1]: Now running on CPU0
      Thread[5]: Now running on CPU0
      Thread[3]: Now running on CPU3
      Thread[3]: Now running on CPU0
      Thread[4]: Now running on CPU2
      Thread[3]: Done
      Thread[4]: Now running on CPU0
      Thread[4]: Done
      Thread[7]: Done
      Thread[2]: Done
      Thread[5]: Now running on CPU2
      Thread[8]: Now running on CPU1
      Thread[8]: Done
      Thread[6]: Now running on CPU3
      Thread[5]: Done
      Thread[1]: Done
        Main[0]: Now running on CPU1
        Main[0]: Thread 1 completed with result=0
        Main[0]: Thread 2 completed with result=0
        Main[0]: Thread 3 completed with result=0
        Main[0]: Thread 4 completed with result=0
        Main[0]: Thread 5 completed with result=0
      Thread[6]: Done
        Main[0]: Now running on CPU0
        Main[0]: Thread 6 completed with result=0
        Main[0]: Thread 7 completed with result=0
        Main[0]: Thread 8 completed with result=0
      nsh>

    NOTES:

    1. See the notes for the nsh configuration.  Since this configuration
       is essentially the same all of those comments apply.

    2. See the STATUS and SMP sections above for detailed SMP-related
       issues.  There are a some major problems with the current SMP
       implementation.

  knsh
  ---
    This is a configuration of testing the BUILD_KERNEL configuration.

    $ cd nuttx
    $ ./tools/configure.sh sabre-6quad:knsh
    $ make V=1 -j7
    $ make export V=1
    $ cd ../apps
    $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
    $ make import V=1
    $ cd ../nuttx
    $ qemu-system-arm -semihosting -M sabrelite -m 1024 -smp 4 -nographic -kernel ./nuttx

    NuttShell (NSH) NuttX-10.2.0
    nsh> uname -a
    NuttX 10.2.0 31283faf71 Mar  1 2022 19:52:48 arm sabre-6quad
    nsh> ps
    PID GROUP PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK   USED  FILLED COMMAND
      0     0   0 FIFO     Kthread N-- Ready              00000000 002024 000984  48.6%  Idle Task
      1     1 100 RR       Task    --- Running            00000000 002016 001232  61.1%  /system/bin/init
    nsh> free
                       total       used       free    largest  nused  nfree
            Umem:    1048224       3728    1044496    1038304      6      2
            Kmem: 1065201424      10720 1065190704 1065190704     30      1
            Page:  134217728    1122304  133095424  133095424
    nsh> /system/bin/hello
    Hello, World!!
    nsh> /system/bin/getprime
    Set thread priority to 10
    Set thread policy to SCHED_RR
    Start thread #0
    thread #0 started, looking for primes < 10000, doing 10 run(s)
    thread #0 finished, found 1230 primes, last one was 9973
    Done
    /system/bin/getprime took 1850 msec

  knsh_smp
  --------
    This is a configuration of testing the BUILD_KERNEL and SMP configuration.

    $ cd nuttx
    $ ./tools/configure.sh sabre-6quad:knsh_smp
    $ make
    $ make export
    $ cd ../apps
    $ ./tools/mkimport.sh -z -x ../nuttx/nuttx-export-*.tar.gz
    $ make import
    $ cd ../nuttx
    $ qemu-system-arm -semihosting -M sabrelite -m 1024 -smp 4 -nographic -kernel ./nuttx

    NuttShell (NSH) NuttX-10.3.0-RC2
    nsh> uname -a
    NuttX 10.3.0-RC2 d9e95a8f9b-dirty May 26 2022 23:39:42 arm sabre-6quad
    nsh> ps
      PID GROUP CPU PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK   USED  FILLED COMMAND
        0     0   0   0 FIFO     Kthread N-- Assigned           00000000 002024 000880  43.4%  CPU0 IDLE
        1     1   1   0 FIFO     Kthread N-- Running            00000000 002024 000664  32.8%  CPU1 IDLE
        2     2   2   0 FIFO     Kthread N-- Running            00000000 002024 000664  32.8%  CPU2 IDLE
        3     3   3   0 FIFO     Kthread N-- Running            00000000 002024 000664  32.8%  CPU3 IDLE
        4     4   0 100 RR       Task    --- Running            00000000 004064 001116  27.4%  /system/bin/init
    nsh> free
                       total       used       free    largest  nused  nfree
            Umem:    1048224       5872    1042352    1036256      9      2
            Kmem: 1065247648      12800 1065234848 1065234768     45      3
            Page:  134217728    1101824  133115904  133115904
    nsh> hello
    Hello, World!!
    nsh> getprime 4
    Set thread priority to 10
    Set thread policy to SCHED_RR
    Start thread #0
    Start thread #1
    thread #1 started, looking for primes < 10000, doing 10 run(s)
    Start thread #2
    thread #2 started, looking for primes < 10000, doing 10 run(s)
    Start thread #3
    thread #3 started, looking for primes < 10000, doing 10 run(s)
    thread #0 started, looking for primes < 10000, doing 10 run(s)
    thread #1 finished, found 1230 primes, last one was 9973
    thread #2 finished, found 1230 primes, last one was 9973
    thread #3 finished, found 1230 primes, last one was 9973
    thread #0 finished, found 1230 primes, last one was 9973
    Done
    getprime took 1570 msec
    nsh>
