# Getting Started with nsh configuration
1. Download and install toolchain

  $ curl https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.3.0-2019.08.0-x86_64-linux-ubuntu14.tar.gz

2. Download flash tools
  There are multiple tools that can be used, but option (a) is recommended.
  a. blflash -- cross platform open-source (recommended)
      curl -L -o blflash https://github.com/spacemeowx2/blflash/releases/download/v0.3.0/blflash-linux-amd64
      chmod +x ./blflash

  b. Bouffalo Lab flash tools (official)
      https://github.com/bouffalolab/flash_tools.git

3. Configure and build NuttX

  $ mkdir ./nuttx; cd ./nuttx
  $ git clone https://github.com/apache/incubator-nuttx.git nuttx
  $ git clone https://github.com/apache/incubator-nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh bl602evb:nsh
  $ make -j

4. Connect bl602 and computer via USB

5. Flash
  Use option (a) or (b) based on tool selection in section 2.
  a. Using blflash
    Place the board in bootloader mode
     bl602evb: Jumper IO8 to HI and press reset
     DT-BL10: Press and hold D8, press and release EN, release D8
    
    $ blflash flash nuttx.bin --port /dev/ttyUSB0
      [INFO  blflash::flasher] Start connection...
      [TRACE blflash::flasher] 5ms send count 55
      [TRACE blflash::flasher] handshake sent elapsed 159.674µs
      [INFO  blflash::flasher] Connection Succeed
      [INFO  blflash] Bootrom version: 1
      [TRACE blflash] Boot info: BootInfo { len: 14, bootrom_version: 1, otp_info: [0, 0, 0, 0, 3, 0, 0, 0, a4, 9b, 2, 42, e8, b4, 1d, 0] }
      [INFO  blflash::flasher] Sending eflash_loader...
      [INFO  blflash::flasher] Finished 2.563450723s 11.15KB/s
      [TRACE blflash::flasher] 5ms send count 500
      [TRACE blflash::flasher] handshake sent elapsed 5.065786ms
      [INFO  blflash::flasher] Entered eflash_loader
      [INFO  blflash::flasher] Skip segment addr: 0 size: 47504 sha256 matches
      [INFO  blflash::flasher] Skip segment addr: e000 size: 272 sha256 matches
      [INFO  blflash::flasher] Skip segment addr: f000 size: 272 sha256 matches
      [INFO  blflash::flasher] Skip segment addr: 10000 size: 351584 sha256 matches
      [INFO  blflash::flasher] Skip segment addr: 1f8000 size: 5671 sha256 matches
      [INFO  blflash] Success

  b Using BL flash_tools
    Run flash tools, select the nuttx.bin generated in the previous step in the
    Firmware bin field, and refer to the document for the settings of the remaining fields.

6. Run connect terminal to UART  (default baudrate 2000000)
    $ screen /dev/ttyUSB0 2000000
7. Reset Device
    bl602evb: Press reset
    DT-BL10: Press and release EN
8. NSH should appear on console


# Using openocd and gdb:
This guide is focused on the bl602evb board. See the reference manual for
how to connect to the JTAG interface for other boards.

1. Make sure you are running a recently built version of openocd.  The packaged
versions are very old, and will likely not work. https://github.com/ntfreak/openocd


2. Create a openocd-bl602.cfg file
##############################################################################
adapter driver ftdi
ftdi_vid_pid 0x0403 0x6010
ftdi_channel 1

transport select jtag
adapter speed 2000

ftdi_layout_init 0x00f8 0x00fb
set _CHIPNAME riscv
jtag newtap $_CHIPNAME cpu -irlen 5 -expected-id 0x20000c05

set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME.0 riscv -chain-position $_TARGETNAME
$_TARGETNAME.0 configure -work-area-phys 0x22020000 -work-area-size 0x10000 -work-area-backup 1

echo "Ready for Remote Connections"

$_TARGETNAME.0 configure -event reset-assert-pre {
    echo "reset-assert-pre"
    adapter speed 100
}

$_TARGETNAME.0 configure -event reset-deassert-post {
    echo "reset-deassert-post"
    adapter speed 4000
    reg mstatus 0x80000000
    reg pc 0x21000000
}

$_TARGETNAME.0 configure -event reset-init {
    echo "reset-init"
    adapter speed 4000
}

gdb_memory_map enable
gdb_flash_program disable

riscv set_prefer_sba on
riscv set_command_timeout_sec 1

init
reset init
##############################################################################

3. Create a openocd.gdb configuration
##############################################################################
target extended-remote :3333

set print asm-demangle on
set backtrace limit 32

mem 0x22008000 0x22014000 rw
mem 0x42008000 0x42014000 rw
mem 0x22014000 0x22020000 rw
mem 0x42014000 0x42020000 rw
mem 0x22020000 0x22030000 rw
mem 0x42020000 0x42030000 rw
mem 0x22030000 0x2204C000 rw
mem 0x42030000 0x4204C000 rw
mem 0x23000000 0x23400000 ro
mem 0x40000000 0x40010000 rw

break __start
stepi

##############################################################################

3. Connect openocd
$ openocd  -f openocd-bl602.cfg 
Open On-Chip Debugger 0.10.0+dev-01514-ga8edbd020-dirty (2020-11-19-20:11)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Ready for Remote Connections
Info : clock speed 2000 kHz
Info : JTAG tap: riscv.cpu tap/device found: 0x20000c05 (mfg: 0x602 (<unknown>), part: 0x0000, ver: 0x2)
Info : datacount=1 progbufsize=2
Info : Disabling abstract command reads from CSRs.
Info : Examined RISC-V core; found 1 harts
Info :  hart 0: XLEN=32, misa=0x40801125
Info : starting gdb server for riscv.cpu.0 on 3333
Info : Listening on port 3333 for gdb connections
Info : JTAG tap: riscv.cpu tap/device found: 0x20000c05 (mfg: 0x602 (<unknown>), part: 0x0000, ver: 0x2)
reset-assert-pre
reset-deassert-post
Info : Disabling abstract command writes to CSRs.
reset-init
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections

4. Connect gdb
  This is a short demo placing a breakpoint at the entry to nsh.
  NOTE: openocd+gdb cannot be used at this time to flash the device.

  $ riscv64-unknown-elf-gdb nuttx -x openocd-bl602.gdb
  Reading symbols from nuttx...
  0x21000000 in ?? ()
  Breakpoint 1 at 0x2300000a: file chip/bl602_entry.S, line 41.
  Note: automatically using hardware breakpoints for read-only addresses.
  0x21000004 in ?? ()
  (gdb) b nsh_main
  Breakpoint 2 at 0x23003f32: file nsh_main.c, line 123.
  (gdb) mon reset halt
  JTAG tap: riscv.cpu tap/device found: 0x20000c05 (mfg: 0x602 (<unknown>), part: 0x0000, ver: 0x2)
  reset-assert-pre
  reset-deassert-post
  (gdb) c
  Continuing.

  Breakpoint 1, bl602_start () at chip/bl602_entry.S:41
  41	    li t0, MSTATUS_MIE
  (gdb) c
  Continuing.

  Breakpoint 2, nsh_main (argc=1, argc@entry=<error reading variable: value has been optimized out>, argv=0x420134c0, argv@entry=<error reading variable: value has been optimized out>)
      at nsh_main.c:123
  123	  sched_getparam(0, &param);
  (gdb) bt
  #0  nsh_main (argc=1, argc@entry=<error reading variable: value has been optimized out>, argv=0x420134c0, argv@entry=<error reading variable: value has been optimized out>) at nsh_main.c:123
  #1  0x230030f0 in nxtask_startup (entrypt=<optimized out>, argc=<optimized out>, argv=<optimized out>) at sched/task_startup.c:165
  #2  0x23001362 in nxtask_start () at task/task_start.c:144
  #3  0x00000000 in ?? ()
  Backtrace stopped: frame did not save the PC