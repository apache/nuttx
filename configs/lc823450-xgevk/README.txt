README
^^^^^^

README for NuttX port to the LC823450XGEVK board.

The board information is available at

  http://www.onsemi.com/PowerSolutions/evalBoard.do?id=LC823450XGEVK

LC823450 related documents are available at

  http://www.onsemi.com/PowerSolutions/supportDoc.do?type=AppNotes&rpn=LC823450

OpenOCD for NuttX thread and LC823450 support is available at

  https://github.com/sony/openocd-nuttx/wiki

MakeIPL2 Tool for eMMC boot is available at

  http://www.onsemi.com/PowerSolutions/supportDoc.do?type=software&rpn=LC823450

This port is intended to test LC823450 features including SMP.
Supported peripherals:
UART, TIMER, RTC, GPIO, DMA, I2C, SPI, LCD, eMMC, USB, WDT, ADC, Audio.

Settings
^^^^^^^^

1. eMMC boot and SRAM boot via openocd are supported.

If you do SRAM boot via openocd+gdb, please specify hookpost-load in .gdbinit
to set MSP (main stack pointer) as follows.

  define hookpost-load
    print *(uint32_t *)0x02040000
    set $sp=$
  end

2. If SWD connection is lost, please specify lower adaptor clock.
3. Both CPUs are running at 160MHz.
4. Internal SRAMs (seg0 to seg5) are used.
5. Serial console can be used via external USB-UART (115200/8/N/1).
6. Interrupt handlers except for inter-cpu are handled on CPU0.

SMP related Status
^^^^^^^^^^^^^^^^^^

CPU activities are shown at D9 (CPU0) and D10 (CPU1) respectively.

Currently all applications except for ostest work in SMP mode but might stop
due to deadlocks or ASSERT(). For a workaround, please try

$ cd apps; git diff
diff --git a/examples/ostest/waitpid.c b/examples/ostest/waitpid.c
index 687f50ca..8418eff8 100644
--- a/examples/ostest/waitpid.c
+++ b/examples/ostest/waitpid.c
@@ -54,7 +54,7 @@
  ****************************************************************************/

 #define RETURN_STATUS 14
-#define NCHILDREN     3
+#define NCHILDREN     2
 #define PRIORITY      100

 /****************************************************************************

Other Status
^^^^^^^^^^^^

1. nsh built-in commands such as ps, free are available.

NuttShell (NSH)
nsh> ps
  PID GROUP CPU PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK COMMAND
    0     0   0   0 FIFO     Kthread N-- Assigned           00000000 000000 CPU0 IDLE
    1     0   1   0 FIFO     Kthread N-- Running            00000000 002044 CPU1 IDLE
    3     1   0 100 FIFO     Task    --- Running            00000000 003052 init
nsh> free
             total       used       free    largest
Mem:       1027024      13136    1013888    1013888

2. date command can be used to get/set RTC date and time.

nsh> date
Oct 03 00:00:55 2013
nsh> date -s "Mar 31 12:34:56 2017"
nsh> date
Mar 31 12:34:56 2017

3. i2c app can be used to test I2C buses.

nsh> i2c get -b 1 -a 18 -r 0
READ Bus: 1 Addr: 18 Subaddr: 00 Value: f9

4. nxhello app can be used to test LCD via SPI.

nsh> nxhello
nxhello_initialize: Initializing LCD
nxhello_initialize: Open NX
nxhello_main: NX handle=20096f0
nxhello_main: Set background color=0
nxhello_main: Screen resolution (128,48)
nxhello_hello: Position (31,20)
nxhello_main: Close NX

5. eMMC can be accessed via /dev/mtdblock0pX

nsh> mkfatfs -F 32 /dev/mtdblock0p10
nsh> mount -t vfat /dev/mtdblock0p10 /mnt/sd0
nsh> df
  Block  Number
  Size   Blocks     Used Available Mounted on
 16384   453025        2    453023 /mnt/sd0
     0        0        0         0 /proc
nsh> ls /mnt/sd0
/mnt/sd0:
nsh> ps > /mnt/sd0/ps.txt
nsh> ls /mnt/sd0
/mnt/sd0:
 ps.txt

Micro SD slot on the board can be used via /dev/mtdblock1.
Please note that card hotplugging is not supported.

6. USB Mass Storage Class support

nsh> msconn
nsh> msdis

7. ADC

nsh> adc
adc_main: g_adcstate.count: 1
adc_main: Hardware initialized. Opening the ADC device: /dev/adc0
Sample:
1: channel: 0 value: 366
2: channel: 1 value: 691
3: channel: 2 value: 752
4: channel: 3 value: 963
5: channel: 4 value: 6
6: channel: 5 value: 0

8. WDT

nsh> wdog
  ping elapsed=0
  ping elapsed=500
  ping elapsed=1000
  ping elapsed=1500
  ping elapsed=2000
  ping elapsed=2500
  ping elapsed=3000
  ping elapsed=3500
  ping elapsed=4000
  ping elapsed=4500
  NO ping elapsed=5000
  NO ping elapsed=5500
  NO ping elapsed=6000

9. IPL2 and eMMC boot

IPL2 is the 2nd boot loader based on NuttX and can be built as follows.

  $ make distclean
  $ ./tools/configure.sh lc823450-xgevk/ipl2
  $ make V=1
  $ MakeIPL2 ./nuttx.bin 0 2 0 0 0
  $ cp LC8234xx_17S_start_data.boot_bin /tmp/

To write the IPL2 (LC8234xx_17S_start_data.boot_bin),
firstly build USB configuration image.

  $ make distclean
  $ ./tools/configure.sh lc823450-xgevk/usb
  $ make V=1

Load the nuttx.bin with openocd + gdb

  $ cd openocd-nuttx
  $ ./bootstrap
  $ ./configure
  $ make
  $ sudo ./src/openocd -s ./tcl -f ./tcl/board/lc823450_xgevk.cfg  -c init -c "reset halt"

  $ arm-none-eabi-gdb
  (gdb) target extended-remote :3333
  (gdb) load ./nuttx
  (gdb) symbol-file ./nuttx
  (gdb) c

Start USB MSC to copy nuttx.bin and the IPL2 to the FAT32 partition (/dev/mtdblock0p10)
then dd the files to the kernel partition (/dev/mtdblock0p4) and the IPL2 partition
(/dev/mtdblock0p1) respectively.

  nsh> mkfatfs -F 32 /dev/mtdblock0p10
  nsh> msconn

  $ sudo cp ./nuttx.bin /media/usb0/
  $ sudo cp /tmp/LC8234xx_17S_start_data.boot_bin /media/usb0/
  $ sudo sync

  nsh> msdis
  nsh> mount -t vfat /dev/mtdblock0p10 /mnt/sd0
  nsh> dd if=/mnt/sd0/nuttx.bin of=/dev/mtdblock0p4
  nsh> dd if=/mnt/sd0/LC8234xx_17S_start_data.boot_bin of=/dev/mtdblock0p1
  nsh> reboot

10. Audio playback (WAV/44.1k/16bit/2ch only)

Firstly, please make sure that the jumper pins are set as follows.

  JP1, JP2 => short
  JP3, JP4 => open

To play WAV file on uSD card,

  nsh> mount -t vfat /dev/mtdblock1 /mnt/sd1
  nsh> nxplayer
  nxplayer> play /mnt/sd1/sample.wav
  nxplayer> volume 50

Please note that a WAV file which contains sub-chunks other than "fmt"
and "data" is not supported in pcm_decode.c So, if your wav file contains
meta-data, please remove the sub-chunks before playing.

11. Networking

lc823450/rndis configuration supports networking features with RNDIS.
To use this feature, you have to connect the board to a RNDIS host.
Currently Linux host is only tested but Windows host should work.

If DHCP server is available, you would see ifconfig results like:

nsh> ifconfig
eth0 Link encap:Ethernet HWaddr 00:e0:de:ad:be:ff at UP
     inet addr:192.168.1.244 DRaddr:192.168.1.1 Mask:255.255.255.0


lo   Link encap:Local Loopback at UP
     inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0


             IPv4   TCP   UDP  ICMP
Received     0007  0000  0006  0000
Dropped      0001  0000  0000  0000
  IPv4        VHL: 0000   Frg: 0001
  Checksum   0000  0000  0000  ----
  TCP         ACK: 0000   SYN: 0000
              RST: 0000  0000
  Type       0000  ----  ----  0000
Sent         0003  0000  0003  0000
  Rexmit     ----  0000  ----  ----

12. DVFS (Dynamic Voltage and Frequency Scaling)

lc823450-xgevk/audio and rndis configurations support DVFS.
You can check the status via /proc/dvfs

nsh> cat /proc/dvfs
cur_freq 160
enable 0

By default, DVFS is disabled. To enable,

nsh> echo "enable 1" > /proc/dvfs

In addition, you can change CPU frequency to 160/80/40 manually.
To change the frequency, enable the DVFS first then do the following.

nsh> echo "cur_freq 80" > /proc/dvfs.

If you want to run in autonomous mode,

nsh> echo "auto 1" > /proc/dvfs.

In autonomous mode, you don't need to set cur_freq. Instead,
cur_freq will show the current CPU frequency.

NOTE: Currently Vdd1 is fixed to 1.2V which will be changed
in the future version.

TODO
^^^^

The following features will be supported.
Accelerometer, etc.
