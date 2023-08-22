=======
emw3162
=======

EMW3162 board (https://www.waveshare.com/EMW3162.htm) features the
STM32F205RG MCU and Broadcom BCM43362KUBG Wi-Fi chip.
The STM32F205RG is a 120 MHz Cortex-M3 operation with 1MB Flash
memory and 128KB RAM.

Configuring NuttX for the EMW3162 board
=======================================

::

  $ cd nuttx
  $ make apps_distclean
  $ make distclean
  $ ./tools/configure.sh emw3162:wlan

Configuring NuttX to use your Wireless Router (aka Access Point)
================================================================

::

  $ make menuconfig

Browse the menus this way::

  Application Configuration  --->
      NSH Library  --->
          Networking Configuration  --->
              WAPI Configuration  --->
                  (myApSSID) SSID
                  (mySSIDpassphrase) Passprhase

Replace the SSID from myApSSID with your wireless router name and
the Passprhase with your WiFi password.

Exit and save.

Finally just compile NuttX::

  $ make

Programming Flash
=================

Flash memory can be programmed by stlink toolset
(https://github.com/stlink-org/stlink) and ST-LINK V2 programmer
(via SWD interface) as follows::

  $ sudo st-flash write nuttx.bin 0x8000000

NSH via telnet
==============

After you successfully downloaded nuttx.bin, reset the board and it
automatically connects to the corresponding wifi AP. You may login
your router to see its IP address. Assume that it's 192.168.1.111

Open a terminal on your computer and telnet your EMW3162 board::

  $ telnet 192.168.1.111
  Trying 192.168.1.111...
  Connected to 192.168.1.111.
  Escape character is '^]'

  NuttShell (NSH) NuttX-10.1.0-RC1
  nsh>

Serial console configuration
============================

Connect a USB/Serial 3.3V dongle to GND, TXD and RXD pins of EMW3162 board.
Then use some serial console client (minicom, picocom, teraterm, etc) confi-
gured to 115200 8n1 without software or hardware flow control.

Reset the board and you should see NuttX starting in the serial.
