README
======

  This is the README file for the port of NuttX to the TI CC3200 Launchpad.

OpenOCD for Windows
===================

  Get the CC3200 SDK
  ------------------
  Get this from the TI web site. Also get "CC3200 SimpleLink™ Wi-Fi® and
  IoT Solution With MCU LaunchPad™ Getting Started Guide" (SWRU376A)

  Get OpenOCD
  ------------
  The OpenOCD project is here: http://openocd.sourceforge.net/

  I use the pre-built binaries provided by Freddie Chopin that can b
  downloaded here: http://www.freddiechopin.info/

  I used version 0.8.0 which available here:
  http://www.freddiechopin.info/en/articles/34-news/92-openocd-w-wersji-080

  Other versions are available here:
  http://www.freddiechopin.info/en/download/category/4-openocd

  Get Zadig
  ---------
  Unless you are very clever with Windows drivers, then I also recommend
  that you download and install Zadig: http://zadig.akeo.ie/

  Other Stuff
  -----------
  USB Cable, your favorite serial terminal program, NuttX build with
  one of the CC3200 configurations in this diretory.

  Installing
  ----------
  Install the TI CC3200 SDK and OpenOCD.  Zadig is just an binary so there
  is no installation.  Plug in the CC3200 via the USB cable.  You should see
  two new devices in the Windows Device Manager, both called:

    USB <-> JTAG/SWD

  There will be indications on the driver icon that no driver is installed.

  Follow the instructions in the paragraph "Install USB Driver" to install
  the TI USB drivers.  You need to do this twice, once for each device.  Now
  you will have two devices with different names:

    CC3200CP JTAG Port A, and
    CC3200CP UART Port B

  OpenOCD cannot use the TI JTAG drivers.  So we need to replace that port
  (ONLY) with the libusb driver.  Use Zadig to install the libusb driver
  replacing the TI driver for "CC3200CP JTAG Port A".  Now you should have
  the following under "Ports (COM & LPT)":

    CC3200 UART Port B

  And under "Universal Serial Bus Devices", again:

    USB <-> JTAG/SWD

  But this time without the indication that a driver is needed.

  Starting OpenOCD
  ----------------
  These instructions assume that (1) you are using a terminal with a Bash
  shell under Cygwin, (2) that you installed OpenOCD at C:\openocd-0.8.0,
  and (3) you are using a 64-bit windows version.  You will need to make
  minor changes if any of these are not true.

  The script to use with OpenOCD 0.8.0 is provided in
  nuttx/configs/cc3200-launchpad/tools.  Go there and start OpenOCd as
  follow:

    $ cd configs/cc3200-launchpad/tools
    $ /cygdrive/c/openocd-0.8.0/bin-x64/openocd-x64-0.8.0.exe --file cc3200.cfg

  And you should see something like:

    Open On-Chip Debugger 0.8.0 (2014-04-28-08:42)
    Licensed under GNU GPL v2
    For bug reports, read
            http://openocd.sourceforge.net/doc/doxygen/bugs.html
    Info : only one transport option; autoselect 'jtag'
    adapter speed: 1000 kHz
    Info : clock speed 1000 kHz
    Info : JTAG tap: cc3200.jrc tap/device found: 0x0b97c02f (mfg: 0x017, part: 0xb97c, ver: 0x0)
    Info : JTAG tap: cc3200.dap enabled
    Info : cc3200.cpu: hardware has 6 breakpoints, 4 watchpoints

  Open the Serial Terminal
  ------------------------
  Connect the CC3200 board via the USB cabale.  Open the serial terminal
  program using the libusb COM device.  For me this is usually COM6 but
  could be anything.  If you are unsure, remove the CC3200 and see which
  one goes away.

  The serial interface should be configured 115200 8N1.

  Using GDB
  ---------
  Start GDB and connect to OpenOCD:

    $ arm-none-eabi-gdb
    (gdb) target remote localhost:3333

  Load and start the NuttX ELF file (nuttx):

    (gdb) mon reset halt
    (gdb) load nuttx
    (gdb) cont
    (gdb)

  After entering cont(inue), you should see the NSH prompt in the serial
  terminal window:

    C3200 init

    NuttShell (NSH)
    nsh>
