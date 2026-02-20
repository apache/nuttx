=================
ST Nucleo H743ZI2
=================

.. tags:: chip:stm32, chip:stm32h7, chip:stm32h743

This page discusses issues unique to NuttX configurations for the
STMicro NUCLEO-H743ZI2 development board featuring the STM32H743ZI
MCU. The STM32H743ZI is a 400MHz Cortex-M7 operation with 2MBytes Flash
memory and 1MByte SRAM. The board features:

- On-board ST-LINK/V2 for programming and debugging,
- 3 user LEDs
- Two pushbuttons (user[B1] and reset)
- 32.768 kHz crystal oscillator
- USB OTG FS with Micro-AB connectors
- Ethernet connector compliant with IEEE-802.3-2002
- Board connectors:
  - USB with Micro-AB
  - SWD
  - Ethernet RJ45
  - ST Zio connector including Arduino Uno V3
  - ST morpho

Refer to the http://www.st.com website for further information about this
board (search keyword: NUCLEO-H743ZI2)

Serial Console
==============

Many options are available for a serial console via the Morpho connector.
Here two common serial console options are suggested:

1. Arduino Serial Shield.

   If you are using a standard Arduino RS-232 shield with the serial
   interface with RX on pin D0 and TX on pin D1 from USART6:

      ======== ========= =====
      ARDUINO  FUNCTION  GPIO
      ======== ========= =====
      DO RX    USART6_RX PG9
      D1 TX    USART6_TX PG14
      ======== ========= =====

2. Nucleo Virtual Console.

   The virtual console uses Serial Port 3 (USART3) with TX on PD8 and RX on
   PD9.

      ================= ===
      VCOM Signal       Pin
      ================= ===
      SERIAL_RX         PD9
      SERIAL_TX         PD8
      ================= ===

   These signals are internally connected to the on board ST-Link.

   The Nucleo virtual console is the default serial console in all
   configurations unless otherwise stated in the description of the
   configuration.

Configurations
==============

nsh:
----

This configuration provides a basic NuttShell configuration (NSH)
for the Nucleo-H743ZI.  The default console is the VCOM on USART3.

jumbo:
------

This configuration enables many Apache NuttX features.  This is
mostly to help provide additional code coverage in CI, but also
allows for a users to see a wide range of features that are
supported by the OS.

Some highlights:
  NSH:
    - Readline with tab completion
    - Readline command history

  Performance and Monitoring:
    - RAM backed syslog
    - Syslog with process name, priority, and timestamp
    - Process Snapshot with stack usage, cpu usage, and signal information
    - Interrupt Statistics
    - procfs filesystem (required for ifconfig, ifup/ifdown)

  Networking:
    - IPv4 Networking
    - Ethernet
    - DHCP Client
    - iperf
    - telnet daemon

  File Systems:
    - FAT filesystem
    - LittleFS
    - RAM MTD device

  Testing:
    - OS Test with FPU support
    - Filesystem testing

  USB Host:
    - USB Hub support
    - Mass Storage Device
    - Trace Monitoring


..
   ADE
   telnetd [6:100]
   
   NuttShell (NSH)
   nsh>
   nsh> uname -a
   NuttX  0.0.0 643f33934e-dirty Feb 20 2026 14:50:28 arm nucleo-h743zi2
   nsh> ?
   help usage:  help [-v] [<cmd>]
   
       .           cp          expr        mkrd        route       truncate
       [           cmp         false       mount       set         uname
       ?           dirname     fdinfo      mv          kill        umount
       addroute    date        free        nslookup    pkill       unset
       alias       delroute    help        pidof       sleep       uptime
       unalias     df          hexdump     printf      usleep      watch
       arp         dmesg       ifconfig    ps          source      wget
       basename    echo        irqinfo     pwd         test        xd
       break       env         ls          reboot      top         wait
       cat         exec        mkdir       rm          time
       cd          exit        mkfatfs     rmdir       true
   
   Builtin Apps:
       dd          hidkbd      nsh         renew       telnetd
       fstest      iperf       ostest      sh
       getprime    netcat      ping        tc
   nsh> ps
     TID   PID  PPID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK            STACK    USED FILLED    CPU COMMAND
       0     0     0   0 FIFO     Kthread   - Ready              0000000000000000 0001000 0000544  54.4%  100.0% Idle_Task
       1     0     0 224 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% hpwork 0x24000120 0x24000168
       2     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% lpwork 0x240000bc 0x24000104
       3     3     0 100 RR       Task      - Running            0000000000000000 0004048 0001768  43.6%   0.0% nsh_main
       4     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0002008 0000856  42.6%   0.0% usbhost
       5     0     0  50 RR       Kthread   - Waiting  Signal    0000000000000000 0004048 0000536  13.2%   0.0% USB_Monitor
       6     6     0 100 RR       Task      - Waiting  Semaphore 0000000000000000 0002008 0000880  43.8%   0.0% telnetd
   nsh> free
         total       used       free    maxused    maxfree  nused  nfree name
        956604     158372     798232     158768     464552     62      5 Umem
   nsh> irqinfo
   IRQ HANDLER  ARGUMENT    COUNT    RATE    TIME
    11 080012c1 00000000       1757   19.719    0
    15 0800804d 00000000       8910  100.000    1
    55 080007a1 24000000       2338   26.240    5
    77 0800849d 00000000         30    0.336    1
   117 0803bbb1 00000000        206    2.316 21558
   nsh> ifconfig
   eth0    Link encap:Ethernet HWaddr ea:63:b9:20:1d:46 at RUNNING mtu 1486
           inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0
   
   lo      Link encap:Local Loopback at RUNNING mtu 1518
           inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0
   
                IPv4   TCP   UDP  ICMP
   Received     000a  0000  000a  0000
   Dropped      0000  0000  0000  0000
     IPv4        VHL: 0000   Frg: 0000
     Checksum   0000  0000  0000  ----
     TCP         ACK: 0000   SYN: 0000
                 RST: 0000  0000
     Type       0000  ----  ----  0000
   Sent         0000  0000  0000  0000
     Rexmit     ----  0000  ----  ----
   nsh> renew eth0
   nsh> ifconfig eth0
   eth0    Link encap:Ethernet HWaddr ea:63:b9:20:1d:46 at RUNNING mtu 1486
           inet addr:192.168.3.120 DRaddr:192.168.3.1 Mask:255.255.255.0
   
   nsh> # telnet to board here
   nsh> ifconfig
   eth0    Link encap:Ethernet HWaddr ea:63:b9:20:1d:46 at RUNNING mtu 1486
           inet addr:192.168.3.120 DRaddr:192.168.3.1 Mask:255.255.255.0
   
   lo      Link encap:Local Loopback at RUNNING mtu 1518
           inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0
   
                IPv4   TCP   UDP  ICMP
   Received     004b  0037  0013  0001
   Dropped      0000  0000  0000  0000
     IPv4        VHL: 0000   Frg: 0000
     Checksum   0000  0000  0000  ----
     TCP         ACK: 0000   SYN: 0000
                 RST: 0000  0000
     Type       0000  ----  ----  0000
   Sent         0039  0036  0002  0001
     Rexmit     ----  0000  ----  ----
   nsh> ping www.google.com
   PING 142.250.191.4 56 bytes of data
   56 bytes from 142.250.191.4: icmp_seq=0 time=20.0 ms
   56 bytes from 142.250.191.4: icmp_seq=1 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=2 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=3 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=4 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=5 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=6 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=7 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=8 time=10.0 ms
   56 bytes from 142.250.191.4: icmp_seq=9 time=20.0 ms
   10 packets transmitted, 10 received, 0% packet loss, time 10100 ms
   rtt min/avg/max/mdev = 10.000/12.000/20.000/4.000 ms
   nsh> ls /dev
   /dev:
    console
    kmsg
    null
    rammtd
    sda
    telnet
    ttyS0
    zero
   nsh> ls /mnt
   /mnt:
    lfs/
   nsh> echo "This will go away on reboot." > /mnt/lfs/afile
   nsh> cat /mnt/lfs/afile
   This will go away on reboot.
   nsh> mount -t vfat /dev/sda /mnt/sda
   nsh> echo "This will stay on the USB drive" > /mnt/sda/afile
   nsh> ls /mnt/sda
   /mnt/sda:
    GARMIN/
    afile
   nsh> rebootADE
   telnetd [6:100]
   
   NuttShell (NSH)
   nsh> ls /mnt/lfs
   /mnt/lfs:
    .
    ..
   nsh> ls /mnt
   /mnt:
    lfs/
   nsh> mount -t vfat /dev/sda /mnt/sda
   nsh> ls /mnt/sda
   /mnt/sda:
    GARMIN/
    afile
   nsh> cat /mnt/sda/afile
   This will stay on the USB drive
   nsh> buttons
   nsh: buttons: command not found
   nsh> ADE
   ADE
   telnetd [6:100]
   
   NuttShell (NSH)
   nsh> uname -a
   NuttX  0.0.0 d147177c62 Feb 20 2026 15:18:50 arm nucleo-h743zi2
   nsh> ?
   help usage:  help [-v] [<cmd>]
   
       .           cp          expr        mkrd        route       truncate
       [           cmp         false       mount       set         uname
       ?           dirname     fdinfo      mv          kill        umount
       addroute    date        free        nslookup    pkill       unset
       alias       delroute    help        pidof       sleep       uptime
       unalias     df          hexdump     printf      usleep      watch
       arp         dmesg       ifconfig    ps          source      wget
       basename    echo        irqinfo     pwd         test        xd
       break       env         ls          reboot      top         wait
       cat         exec        mkdir       rm          time
       cd          exit        mkfatfs     rmdir       true
   
   Builtin Apps:
       buttons     getprime    netcat      ping        tc
       dd          hidkbd      nsh         renew       telnetd
       fstest      iperf       ostest      sh
   nsh> ls
   /:
    dev/
    mnt/
    proc/
   nsh> free
         total       used       free    maxused    maxfree  nused  nfree name
        956492     158452     798040     158848     464440     64      5 Umem
   nsh> irqinfo
   IRQ HANDLER  ARGUMENT    COUNT    RATE    TIME
    11 080012f1 00000000       1073   10.277    0
    15 0800807d 00000000      10440  100.000    1
    55 080007a1 24000000       1380   13.218    5
    77 080084cd 00000000         36    0.344    1
   117 0803c4ed 00000000        206    1.976 21558
   nsh> ifconfig
   eth0    Link encap:Ethernet HWaddr ea:63:b9:20:1d:46 at RUNNING mtu 1486
           inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0
   
   lo      Link encap:Local Loopback at RUNNING mtu 1518
           inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0
   
                IPv4   TCP   UDP  ICMP
   Received     000d  0000  000d  0000
   Dropped      0000  0000  0000  0000
     IPv4        VHL: 0000   Frg: 0000
     Checksum   0000  0000  0000  ----
     TCP         ACK: 0000   SYN: 0000
                 RST: 0000  0000
     Type       0000  ----  ----  0000
   Sent         0000  0000  0000  0000
     Rexmit     ----  0000  ----  ----
   nsh> renew eth0
   nsh> ifconfig eth0
   eth0    Link encap:Ethernet HWaddr ea:63:b9:20:1d:46 at RUNNING mtu 1486
           inet addr:192.168.3.120 DRaddr:192.168.3.1 Mask:255.255.255.0
   
   nsh> ping www.google.com
   PING 142.250.217.132 56 bytes of data
   56 bytes from 142.250.217.132: icmp_seq=0 time=20.0 ms
   56 bytes from 142.250.217.132: icmp_seq=1 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=2 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=3 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=4 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=5 time=20.0 ms
   56 bytes from 142.250.217.132: icmp_seq=6 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=7 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=8 time=10.0 ms
   56 bytes from 142.250.217.132: icmp_seq=9 time=10.0 ms
   10 packets transmitted, 10 received, 0% packet loss, time 10100 ms
   rtt min/avg/max/mdev = 10.000/12.000/20.000/4.000 ms
   nsh> ls /dev
   /dev:
    buttons
    console
    kmsg
    null
    rammtd
    sda
    telnet
    ttyS0
    zero
   nsh> ls /mnt
   /mnt:
    lfs/
   nsh> echo "This will go away on reboot." > /mnt/lfs/afile
   nsh> cat /mnt/lfs/afile
   This will go away on reboot.
   nsh> mount -t vfat /dev/sda /mnt/sda
   nsh> echo "This will stay on the USB drive" > /mnt/sda/afile
   nsh> ls /mnt/sda
   /mnt/sda:
    GARMIN/
    afile
   nsh> rebootADE
   telnetd [6:100]
   
   NuttShell (NSH)
   nsh> ls /mnt/lfs
   /mnt/lfs:
    .
    ..
   nsh> ls /mnt
   /mnt:
    lfs/
   nsh> mount -t vfat /dev/sda /mnt/sda
   nsh> ls /mnt/sda
   /mnt/sda:
    GARMIN/
    afile
   nsh> cat /mnt/sda/afile
   This will stay on the USB drive
   nsh> buttons
   buttons_main: Starting the button_daemon
   buttons_main: button_daemon started
   button_daemon: Running
   button_daemon: Opening /dev/buttons
   button_daemon: Supported BUTTONs 0x01
   nsh> B1 was pressed
   B1 was released
   B1 was pressed
   B1 was released
   
   nsh> ps
     TID   PID  PPID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK            STACK    USED FILLED    CPU COMMAND
       0     0     0   0 FIFO     Kthread   - Ready              0000000000000000 0001000 0000544  54.4%  100.0% Idle_Task
       1     0     0 224 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% hpwork 0x24000120 0x24000168
       2     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% lpwork 0x240000bc 0x24000104
       3     3     0 100 RR       Task      - Running            0000000000000000 0004048 0001768  43.6%   0.0% nsh_main
       4     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0002008 0000856  42.6%   0.0% usbhost
       5     0     0  50 RR       Kthread   - Waiting  Signal    0000000000000000 0004048 0000816  20.1%   0.0% USB_Monitor
       6     6     0 100 RR       Task      - Waiting  Semaphore 0000000000000000 0002008 0000880  43.8%   0.0% telnetd
       8     8     0 100 RR       Task      - Waiting  Signal    0000000000000000 0004048 0000712  17.5%   0.0% button_daemon
   nsh>
   ```

   ```
   peter@legion:~$ telnet 192.168.3.120
   Trying 192.168.3.120...
   Connected to 192.168.3.120.
   Escape character is '^]'.
   
   NuttShell (NSH)
   nsh> ps
   nsh: &ps: command not found
   nsh> ps
     TID   PID  PPID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK            STACK    USED FILLED    CPU COMMAND
       0     0     0   0 FIFO     Kthread   - Ready              0000000000000000 0001000 0000544  54.4%  100.0% Idle_Task
       1     0     0 224 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% hpwork 0x24000120 0x24000168
       2     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0001976 0000592  29.9%   0.0% lpwork 0x240000bc 0x24000104
       3     3     0 100 RR       Task      - Waiting  Semaphore 0000000000000000 0004048 0001384  34.1%   0.0% nsh_main
       4     0     0 100 RR       Kthread   - Waiting  Semaphore 0000000000000000 0002008 0000856  42.6%   0.0% usbhost
       5     0     0  50 RR       Kthread   - Waiting  Signal    0000000000000000 0004048 0000536  13.2%   0.0% USB_Monitor
       6     6     0 100 RR       Task      - Waiting  Semaphore 0000000000000000 0002008 0000880  43.8%   0.0% telnetd
       8     8     0 100 RR       Task      - Running            0000000000000000 0002000 0001864  93.2%!  0.0% Telnet_session
   nsh> ^]
   telnet> quit
   Connection closed.
   peter@legion:~$ 
   ```
