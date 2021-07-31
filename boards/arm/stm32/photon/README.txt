README
======

  This README discusses issues unique to NuttX configurations for the
  Particle.io Photon board featuring the STM32F205RG MCU.
  The STM32F205RG is a 120 MHz Cortex-M3 operation with 1MB Flash
  memory and 128KB RAM. The board includes a Broadcom BCM43362 WiFi.

Contents
========

  - Selecting the Photon board on NuttX
  - Configuring NuttX to use your Wireless Router (aka Access Point)
  - Flashing NuttX in the Photon board
  - Serial console configuration

Selecting the Photon board on NuttX
===================================

  NOTICE: We will not discuss about toolchains and environment configuration
  here, please take a look at STM32F4Discovery board README or other
  STM32 board because it should work for Photon board as well.

  Let us to consider that you cloned the nuttx and apps repositories, then
  follow these steps:

  1) Clear your build system before to start:

     $ make apps_distclean
     $ make distclean

  2) Enter inside nuttx/tools and configure to use the Photon board:

     $ cd nuttx
     $ tools/configure.sh photon:wlan

Configuring NuttX to use your Wireless Router (aka Access Point)
================================================================

  Since you are already in the root of nuttx/ repository, execute
  make menuconfig to define your Wireless Router and your password:

  $ make menuconfig

  Browser the menus this way:

  Application Configuration  --->
      NSH Library  --->
          Networking Configuration  --->
              WAPI Configuration  --->
                  (myApSSID) SSID
                  (mySSIDpassphrase) Passprhase

  Replace the SSID from myApSSID with your wireless router name and
  the Passprhase with your WiFi password.

  Exit and save your configuration.

  Finally just compile NuttX:

    $ make

Flashing NuttX in the Photon board
==================================

  Connect the Photon board in your computer using a MicroUSB cable. Press and
  hold both board's buttons (SETUP and RESET), then release the RESET button,
  the board will start blinking in the Purple color, waiting until it starts
  blinking in Yellow color. Now you can release the SETUP button as well.

  1) You can verify if DFU mode in your board is working, using this command:

     $ sudo dfu-util -l
     dfu-util 0.8

     Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
     Copyright 2010-2014 Tormod Volden and Stefan Schmidt
     This program is Free Software and has ABSOLUTELY NO WARRANTY
     Please report bugs to dfu-util@lists.gnumonks.org

     Found DFU: [2b04:d006] ver=0200, devnum=15, cfg=1, intf=0, alt=1, name="@DCT Flash   /0x00000000/01*016Kg", serial="00000000010C"
     Found DFU: [2b04:d006] ver=0200, devnum=15, cfg=1, intf=0, alt=0, name="@Internal Flash   /0x08000000/03*016Ka,01*016Kg,01*064Kg,07*128Kg", serial="00000000010C"

  2) Flash the nuttx.bin inside the Internal Flash:

     $ sudo dfu-util -d 2b04:d006 -a 0 -s 0x08020000 -D nuttx.bin

     dfu-util 0.8

     Copyright 2005-2009 Weston Schmidt, Harald Welte and OpenMoko Inc.
     Copyright 2010-2014 Tormod Volden and Stefan Schmidt
     This program is Free Software and has ABSOLUTELY NO WARRANTY
     Please report bugs to dfu-util@lists.gnumonks.org

     dfu-util: Invalid DFU suffix signature
     dfu-util: A valid DFU suffix will be required in a future dfu-util release!!!
     Opening DFU capable USB device...
     ID 2b04:d006
     Run-time device DFU version 011a
     Claiming USB DFU Interface...
     Setting Alternate Setting #0 ...
     Determining device status: state = dfuIDLE, status = 0
     dfuIDLE, continuing
     DFU mode device DFU version 011a
     Device returned transfer size 4096
     DfuSe interface name: "Internal Flash   "
     Downloading to address = 0x08020000, size = 331348
     Download	[=========================] 100%       331348 bytes
     Download done.
     File downloaded successfully

   3) Restore the original firmware

     If you config to use the stock bootloader of Photon, you may
     reload the original firmware with dfu-utils as you like. Otherwise
     you must have backuped the whole image beforehand, and reload it
     via SWD debug port.

NSH via telnet
==============

  After you successfully downloaded nuttx.bin, reset the board and it
  automatically connects to the corresponding wifi AP. You may login
  your router to see its IP address. Assume that it's 192.168.1.111

  Open a terminal on your computer and telnet your Photon:

    $ telnet 192.168.1.111
    Trying 192.168.1.111...
    Connected to 192.168.1.111.
    Escape character is '^]'

    NuttShell (NSH) NuttX-7.24
    nsh>

Serial console configuration
============================

  Connect a USB/Serial 3.3V dongle to GND, TX and RX pins of Photon board.
  Then use some serial console client (minicom, picocom, teraterm, etc) confi-
  gured to 115200 8n1 without software or hardware flow control.

  Reset the board and you should see NuttX starting in the serial.
