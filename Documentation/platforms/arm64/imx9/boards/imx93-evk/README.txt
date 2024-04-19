README.txt
==========

The kit i.MX93 Evaluation Kit has a pre-installed Linux image which contains
u-boot and the i.MX93 reference Linux installation.

u-boot is required to boot NuttX (for now) as it initializes the hardware for
us, i.e. DDR, clocks, I/O muxes etc.

==========================================

How to run nuttx on i.MX93 Evaluation Kit.

==========================================

Below is a set of instructions on how to run NuttX on the i.MX93 EVK

==========================================

Pre-requisites

==========================================

- imx93_ca55.JLinkScript which is a custom file, put it wherever you want

==========================================

U-Boot configuration

==========================================

Two things need to be configured on u-boot before NuttX can be loaded:

- u-boot data cache must be turned off
- u-boot must stop to the u-boot console, i.e. the Linux payload must not be loaded

Manual option:

1. Disable u-boot autostart (needs to be done only once):

    Hit any key to stop autoboot:  0
    u-boot=> setenv bootdelay -1
    u-boot=> saveenv
    Saving Environment to MMC... Writing to MMC(0)... OK
    u-boot=> reset

2. On every boot, the data cache must be disabled for options 2 and 3 to work

    u-boot=> dcache off

Automated option:

1. Replace the default bootcmd to disable dcache automatically:

    u-boot=> setenv bootdelay 0
    u-boot=> setenv bootcmd dcache off
    u-boot=> saveenv
    Saving Environment to MMC... Writing to MMC(0)... OK
    u-boot=> reset

To restore the default bootcmd which starts Linux automatically:

    u-boot=> setenv bootcmd run distro_bootcmd;run bsp_bootcmd
    u-boot=> saveenv
    Saving Environment to MMC... Writing to MMC(0)... OK
    u-boot=> reset

The default bootcmd is:

u-boot=> env print bootcmd
bootcmd=run distro_bootcmd;run bsp_bootcmd

==========================================

Loading and running the NuttX image

==========================================

You have three options:

1 - Load via u-boot from SD-card
2 - Load via gdb
3 - Load via JLink

==========================================

Option 1: load via u-boot from SD-card:

==========================================

1. Build nuttx, and move nuttx.bin to SD-card

2. Load from SD-card and start nuttx payload

    u-boot=> dcache off; fatload mmc 1 0x80000000 nuttx.bin; go 0x80000000

==========================================

Option 2: start via gdb:

==========================================

1. Start JLinkGDBServer

    JLinkGDBServer -device CORTEX-A55 -JLinkScriptFile <path_to>/imx93_ca55.JLinkScript

2. Start gdb

    $ aarch64-none-elf-gdb

 2.1 Attach and load nuttx

    (gdb) target remote localhost:2331
    (gdb) set mem inaccessible-by-default off
    (gdb) load <path_to>/nuttx
    (gdb) monitor go

==========================================

Option 3: load with JLink:

==========================================

1. Start JLink 

    $ JLinkExe -device CORTEX-A55 -if JTAG -jtagconf -1,-1 -speed 4000 -JLinkScriptFile <path_to>/imx93_ca55.JLinkScript

 1.1 Add -AutoConnect 1 to connect automatically

    $ JLinkExe -device CORTEX-A55 -if JTAG -jtagconf -1,-1 -speed 4000 -JLinkScriptFile <path_to>/imx93_ca55.JLinkScript -AutoConnect 1

2. Connect JLink
    
 2.1 Connect to the debugger

    Type "connect" to establish a target connection, '?' for help
    J-Link>connect

    You should now have a JLink prompt.
    
    Cortex-A55 identified.
    J-Link>

3. Load nuttx. Note that JLink expects the .elf extension, the default build output of nuttx is just "nuttx" without the extension, so it must be added to the file...

    J-Link>LoadFile <path_to>/nuttx.elf
