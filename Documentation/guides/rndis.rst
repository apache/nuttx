.. include:: /substitutions.rst
.. _rndis:

How to use RNDIS
================

This guide explains the steps needed to get USB RNDIS working, using
the STM32F4Discovery board as example.

If you don't know RNDIS, it is a way to share Ethernet-like connection
over USB port without using any external device to it. Just a USB cable
between your board (that has USB Device) and your computer.

WARNING: RNDIS is going to be removed from Linux kernel because they
understand it as an unsecure protocol. That said use it aware of this
risk and also be aware that it was tested up to Ubuntu 22.04 LTS and
couldn't work in future versions.

Compiling
---------

#. Configure the RNDIS

   There is a sample configuration to use RNDIS on stm32f4discovery
   board. If your board doesn't save a sample example then you need
   to create a configuration by yourself looking this config.

   Just use ``stm32f4discovery:rndis`` board profile for this purpose. 

    .. code-block:: console

       $ cd nuttx
       $ ./tools/configure.sh stm32f4discovery:rndis

#. Compile

    .. code-block:: console

       $ make -j

Flashing
--------

#. Flash the generated nuttx.bin to your board:

    .. code-block:: console

       $ $ sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
       ...
       Open On-Chip Debugger 0.11.0
       ...
       Info : STLINK V2J14S0 (API v2) VID:PID 0483:3748
       Info : Target voltage: 3.203144
       Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
       Info : starting gdb server for stm32f4x.cpu on 3333
       Info : Listening on port 3333 for gdb connections
       target halted due to debug-request, current mode: Thread 
       xPSR: 0x01000000 pc: 0x08000188 msp: 0x20003f24
       Info : device id = 0x10036413
       Info : flash size = 1024 kbytes
       auto erase enabled
       wrote 262144 bytes from file nuttx.bin in 11.043253s (23.182 KiB/s)
       Info : Listening on port 6666 for tcl connections
       Info : Listening on port 4444 for telnet connections


Setup RNDIS in your computer
----------------------------

   These steps show how to connect your board to your Linux machine.

.. todo:: Add Mac and Windows instructions

#. Reset your board

#. Plug a USB cable from the STM32F4Discovery's microUSB to your computer

#. Confirm that your board was detected as a USB RNDIS device:

    .. code-block:: console

       $ dmesg
       ...
       [ 1099.821480] usb 3-3: new full-speed USB device number 12 using xhci_hcd
       [ 1099.972379] usb 3-3: New USB device found, idVendor=584e, idProduct=5342, bcdDevice= 0.01
       [ 1099.972389] usb 3-3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
       [ 1099.972393] usb 3-3: Product: RNDIS gadget
       [ 1099.972396] usb 3-3: Manufacturer: NuttX
       [ 1099.972398] usb 3-3: SerialNumber: 1234
       [ 1099.988952] usbcore: registered new interface driver cdc_ether
       [ 1099.990144] rndis_host 3-3:1.0: skipping garbage
       [ 1099.990641] rndis_host 3-3:1.0: dev can't take 1558 byte packets (max 660), adjusting MTU to 602
       [ 1099.992089] rndis_host 3-3:1.0 eth0: register 'rndis_host' at usb-0000:00:14.0-3, RNDIS device, a0:e0:de:ad:ca:fe
       [ 1099.992102] usbcore: registered new interface driver rndis_host
       [ 1099.994026] usbcore: registered new interface driver rndis_wlan
       [ 1099.997001] rndis_host 3-3:1.0 enxa0e0deadcafe: renamed from eth0

#. Configure your Linux distro to share network to this USB RNDIS device:

   Click in the top right corner of your Ubuntu and go to:

   NuttX Ethernet -> Wired Settings

   Click in the 'Gear icon' and in the tab "IPv4" select: "Shared to other computers"

   Click on "Apply"

   Disconect and connect the USB cable to force it to get IP.

#. Identify what IP address your board got:

    .. code-block:: console

       $ tail -f /var/log/syslog
       ...
       Jan 28 10:30:24 dev dnsmasq-dhcp[35526]: DHCPDISCOVER(enxa0e0deadcafe) 00:e0:de:ad:ca:fe 
       Jan 28 10:30:24 dev dnsmasq-dhcp[35526]: DHCPOFFER(enxa0e0deadcafe) 10.42.0.86 00:e0:de:ad:ca:fe 
       Jan 28 10:30:24 dev dnsmasq-dhcp[35526]: DHCPREQUEST(enxa0e0deadcafe) 10.42.0.86 00:e0:de:ad:ca:fe 
       Jan 28 10:30:24 dev dnsmasq-dhcp[35526]: DHCPACK(enxa0e0deadcafe) 10.42.0.86 00:e0:de:ad:ca:fe nuttx
       Jan 28 10:30:29 dev systemd[1]: NetworkManager-dispatcher.service: Deactivated successfully.
       ^C

#. Ping this IP to confirm it is working:

    .. code-block:: console

       $ ping 10.42.0.86
       PING 10.42.0.86 (10.42.0.86) 56(84) bytes of data.
       64 bytes from 10.42.0.86: icmp_seq=1 ttl=64 time=0.809 ms
       64 bytes from 10.42.0.86: icmp_seq=2 ttl=64 time=0.849 ms
       ^C
       --- 10.42.0.86 ping statistics ---
       2 packets transmitted, 2 received, 0% packet loss, time 1027ms
       rtt min/avg/max/mdev = 0.809/0.829/0.849/0.020 ms

#. Connect to your board over telnet:

    .. code-block:: console

       $ telnet 10.42.0.86
       Trying 10.42.0.86...
       Connected to 10.42.0.86.
       Escape character is '^]'.

       NuttShell (NSH) NuttX-12.0.0
       nsh> 
