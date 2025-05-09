=========================
Seeed Studio XIAO ESP32C6
=========================

The `Seeed Studio XIAO ESP32C6 <https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/>`_ is a general purpose board supplied by
Seeed Studio and it is compatible with the Espressif ESP32C6 ecosystem, sharing the same MCU as ESP32-C6-DevKitC.

.. figure:: xiao-esp32c6.jpg
   :align: center

Features
========

* Two 32-bit RISC-V processors, with the high-performance one running up to 160 MHz,
  and the low-power one clocking up to 20 MHz
* 512KB of SRAM, and 4MB of on-board flash memory
* USB Type-C interface
* Wireless: Complete 2.4GHz Wi-Fi subsystem;
* BLE: Bluetooth 5.0, Bluetooth mesh
* Zigbee,Thread,IEEE 802.15.4
* 1x UART, 1x I2C, 1x SPI,11x GPIO(PWM), 7x ADC
* 1 RESET button, 1 BOOT button

NSH Console
===========

The NuttShell (NSH) console is available over USB using the CDC/ACM
serial interface. To access the console, connect via a terminal emulator
at 115200 baud, 8 data bits, no parity, and 1 stop bit (115200-8N1).


Buttons
=======

The RESET and BOOT buttons can be used to enter "Bootloader" mode by
press and hold the BOOT key while powering up and then press the RESET key once.

Pin Mapping
===========
Pads numbered anticlockwise from USB connector.

===== ========== ==========
Pad   Signal     Notes
===== ========== ==========
0     GPIO00     D0/A0
1     GPIO01     D1/A1
2     GPIO02     D2/A2
3     GPIO21     D3/A3
4     GPIO22     D4/SDA
5     GPIO23     D5/SCL
6     GPIO16     D6/Default TX for UART0 serial console
7     GPIO17     D7/Default RX for UART0 serial console
8     GPIO19     D8/SCK
9     GPIO20     D9/MISO
10    GPIO18     D10/MOSI
11    3V3        Power output to peripherals
12    Ground
13    VIN        +5V Supply to board
===== ========== ==========

Power Supply
============
The working voltage of the MCU is 3.3V. Voltage input connected to
general I/O pins may cause chip damage if it’s higher than 3.3V.

Installation
============

1. Configure and build NuttX:

.. code-block:: console

  $ git clone https://github.com/apache/nuttx.git nuttx
  $ git clone https://github.com/apache/nuttx-apps.git apps
  $ cd nuttx
  $ make distclean
  $ ./tools/configure.sh esp32c6-xiao:usbnsh
  $ make V=1

2. Connect the Seeed Studio XIAO ESP32C6, and enter "Bootloader" mode,
then, flash the ``nuttx.hex`` file using ``esptool``:
(https://docs.espressif.com/projects/esptool/en/latest/esp32/)

Example command:

.. code-block:: bash

    make flash ESPTOOL_PORT=/dev/ttyACM0 ESPTOOL_BINDIR=./


Configurations
==============

nsh
---

Basic NuttShell configuration using serial (console enabled in UART0, exposed via
pins D6/TX and D7/RX, at 115200 bps).

usbnsh
------
Basic NuttShell configuration using CDC/ACM serial (console enabled in USB Port,
at 115200 bps).

.. code-block:: console

  NuttShell (NSH) NuttX-12.9.0
  nsh> uname -a
  NuttX  12.9.0 ebf883ba72 May  8 2025 17:15:47 risc-v esp32c6-xiao


gpio
----
This configuration enabled NuttShell via USB and enabled gpio example.

Testing gpios:

========   ======   ==========
PIN/GPIO    Mode      Device
========   ======   ==========
D0/GPIO0   Output   /dev/gpio0
D1/GPIO1   Input    /dev/gpio1
========   ======   ==========

.. code-block:: console

  nsh> gpio -o 1 /dev/gpio0
  Driver: /dev/gpio0
    Output pin:    Value=1
    Writing:       Value=1
    Verify:        Value=1
  nsh> 
  nsh> gpio -o 0 /dev/gpio0
  Driver: /dev/gpio0
    Output pin:    Value=1
    Writing:       Value=0
    Verify:        Value=0
  nsh> gpio -w 1 /dev/gpio1
  Driver: /dev/gpio1
    Interrupt pin: Value=0
    Verify:        Value=1

wifi
----
This configuration enables a wlan network interface that can be configured and initialized 
using below commands::

    nsh> ifup wlan0
    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

In this case a connection to AP with SSID ``myssid`` is done, using ``mypasswd`` as
password. IP address is obtained via DHCP using ``renew`` command. You can check
the result by running ``ifconfig`` afterwards.

.. code-block:: console

  NuttShell (NSH) NuttX-12.8.0
  nsh> uname -a
  NuttX  12.9.0 ebf883ba72 May  8 2025 17:15:47 risc-v esp32c6-xiao
  nsh> ?
  help usage:  help [-v] [<cmd>]
  
      .           cp          expr        pkill       pwd         uname       
      [           cmp         false       ls          rm          umount      
      ?           dirname     fdinfo      mkdir       rmdir       unset       
      alias       date        free        mkrd        set         uptime      
      unalias     df          help        mount       sleep       usleep      
      arp         dmesg       hexdump     mv          source      watch       
      basename    echo        ifconfig    nslookup    test        xd          
      break       env         ifdown      pidof       time        wait        
      cat         exec        ifup        printf      true        
      cd          exit        kill        ps          truncate    
  
  Builtin Apps:
      dd           getprime     ostest       rand         sh           
      dumpstack    nsh          ping         renew        wapi         
  nsh> wapi psk wlan0 nuttxpwd 3
  nsh> wapi essid wlan0 nuttxnw 1
  nsh> renew wlan0
  nsh> ifconfig
  wlan0   Link encap:Ethernet HWaddr a0:85:e3:0e:4a:30 at RUNNING mtu 576
          inet addr:192.168.59.144 DRaddr:192.168.59.134 Mask:255.255.255.0
  
  nsh> ping 8.8.8.8
  PING 8.8.8.8 56 bytes of data
  56 bytes from 8.8.8.8: icmp_seq=0 time=50.0 ms
  56 bytes from 8.8.8.8: icmp_seq=1 time=40.0 ms
  56 bytes from 8.8.8.8: icmp_seq=2 time=30.0 ms
  56 bytes from 8.8.8.8: icmp_seq=3 time=60.0 ms
  56 bytes from 8.8.8.8: icmp_seq=4 time=100.0 ms
  56 bytes from 8.8.8.8: icmp_seq=5 time=100.0 ms
  56 bytes from 8.8.8.8: icmp_seq=6 time=140.0 ms
  56 bytes from 8.8.8.8: icmp_seq=7 time=40.0 ms
  56 bytes from 8.8.8.8: icmp_seq=8 time=50.0 ms
  56 bytes from 8.8.8.8: icmp_seq=9 time=30.0 ms
  10 packets transmitted, 10 received, 0% packet loss, time 10100 ms
  rtt min/avg/max/mdev = 30.000/64.000/140.000/34.985 ms
  nsh> nslookup google.com
  Host: google.com Addr: 142.251.128.238
  nsh> nslookup nuttx.apache.org
  Host: nuttx.apache.org Addr: 151.101.2.132