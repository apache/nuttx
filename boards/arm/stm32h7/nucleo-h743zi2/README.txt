README
======

  This README discusses issues unique to NuttX configurations for the
  STMicro NUCLEO-H743ZI2 development board featuring the STM32H743ZI
  MCU. The STM32H743ZI is a 400MHz Cortex-M7 operation with 2MBytes Flash
  memory and 1MByte SRAM. The board features:

  - On-board ST-LINK/V2 for programming and debugging,
  - 3 user LEDs
  - Two pushbuttons (user and reset)
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

Contents
========

  - Serial Console
  - Configurations

Serial Console
==============

  Many options are available for a serial console via the Morpho connector.
  Here two common serial console options are suggested:

  1. Arduino Serial Shield.

    If you are using a standard Arduino RS-232 shield with the serial
    interface with RX on pin D0 and TX on pin D1 from USART6:

      -------- ---------------
               STM32H7
      ARDUINO  FUNCTION  GPIO
      -- ----- --------- -----
      DO RX    USART6_RX PG9
      D1 TX    USART6_TX PG14
      -- ----- --------- -----

  2. Nucleo Virtual Console.

    The virtual console uses Serial Port 3 (USART3) with TX on PD8 and RX on
    PD9.

      ----------------- ---
      VCOM Signal       Pin
      ----------------- ---
      SERIAL_RX         PD9
      SERIAL_TX         PD8
      ----------------- ---

    These signals are internally connected to the on board ST-Link.

  The Nucleo virtual console is the default serial console in all
  configurations unless otherwise stated in the description of the
  configuration.

Configurations
==============

  Configuration Sub-directories
  -----------------------------

  nsh:

    This configuration provides a basic NuttShell configuration (NSH)
    for the Nucleo-H743ZI.  The default console is the VCOM on USART3.

  jumbo:

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

```
NuttShell (NSH) NuttX-10.0.1
nsh> uname -a
NuttX  10.0.1 3ab35e48ff-dirty Mar 28 2021 15:35:29 arm nucleo-h743zi2
nsh> ?
help usage:  help [-v] [<cmd>]

  .         cd        dmesg     hexdump   mkrd      reboot    telnetd   xd        
  [         cp        echo      ifconfig  mh        rm        time      
  ?         cmp       env       irqinfo   mount     rmdir     true      
  addroute  dirname   exec      kill      mv        route     uname     
  arp       date      exit      ls        mw        set       umount    
  basename  dd        false     mb        nslookup  sleep     unset     
  break     delroute  free      mkdir     ps        source    usleep    
  cat       df        help      mkfatfs   pwd       test      wget      

Builtin Apps:
  fstest    getprime  nsh       ostest    ping      renew     sh
nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK   USED  FILLED    CPU COMMAND
    0   0 FIFO     Kthread N-- Ready              00000000 001024 000472  46.0%  100.0% Idle Task
    1 224 RR       Kthread --- Waiting  Signal    00000000 002036 000456  22.3%    0.0% hpwork
    2 100 RR       Kthread --- Waiting  Signal    00000000 002036 000680  33.3%    0.0% lpwork
    3 100 RR       Task    --- Running            00000000 004084 001304  31.9%    0.0% init
    4 100 RR       Kthread --- Waiting  Semaphore 00000000 002036 000752  36.9%    0.0% usbhost
    5  50 RR       Kthread --- Waiting  Signal    00000000 004076 000472  11.5%    0.0% USB Monitor
    6 100 RR       Task    --- Waiting  Semaphore 00000000 002012 000648  32.2%    0.0% Telnet daemon 0x38005600
nsh> free
                     total       used       free    largest
        Umem:       944336     159520     784816     451712
nsh> irqinfo
IRQ HANDLER  ARGUMENT    COUNT    RATE    TIME
  3 080011ed 00000000       1599   20.405    0
 15 08004e0d 00000000       7836  100.000 10000
 55 08000799 24000000       1122   14.318    0
 77 0800580b 00000000         14    0.178    0
117 0802c6f1 00000000       1372   17.544    0
nsh> date; sleep 2; date
Thu, Jan 01 00:01:50 1970
Thu, Jan 01 00:01:52 1970
nsh> ifconfig
eth0    Link encap:Ethernet HWaddr 46:fd:66:78:aa:54 at UP
        inet addr:10.0.0.2 DRaddr:10.0.0.1 Mask:255.255.255.0

lo      Link encap:Local Loopback at UP
        inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0

             IPv4   TCP   UDP  ICMP
Received     0004  0000  0004  0000
Dropped      0000  0000  0000  0000
  IPv4        VHL: 0000   Frg: 0000
  Checksum   0000  0000  0000  ----
  TCP         ACK: 0000   SYN: 0000
              RST: 0000  0000
  Type       0000  ----  ----  0000
Sent         0000  0000  0000  0000
  Rexmit     ----  0000  ----  ----
nsh> renew eth0
nsh> ifconfig
eth0    Link encap:Ethernet HWaddr 46:fd:66:78:aa:54 at UP
        inet addr:192.168.86.249 DRaddr:192.168.86.1 Mask:255.255.255.0

lo      Link encap:Local Loopback at UP
        inet addr:127.0.0.1 DRaddr:127.0.0.1 Mask:255.0.0.0

             IPv4   TCP   UDP  ICMP
Received     0007  0000  0007  0000
Dropped      0000  0000  0000  0000
  IPv4        VHL: 0000   Frg: 0000
  Checksum   0000  0000  0000  ----
  TCP         ACK: 0000   SYN: 0000
              RST: 0000  0000
  Type       0000  ----  ----  0000
Sent         0002  0000  0002  0000
  Rexmit     ----  0000  ----  ----
nsh> ping www.google.com
PING 172.217.14.196 56 bytes of data
56 bytes from 172.217.14.196: icmp_seq=0 time=10 ms
56 bytes from 172.217.14.196: icmp_seq=1 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=2 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=3 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=4 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=5 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=6 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=7 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=8 time=0 ms
56 bytes from 172.217.14.196: icmp_seq=9 time=0 ms
10 packets transmitted, 10 received, 0% packet loss, time 10100 ms
nsh> 
nsh> ls /dev
/dev:
 console
 null
 ramlog
 rammtd
 sda
 telnet
 ttyS0
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
 afile
nsh> reboot

NuttShell (NSH) NuttX-10.0.1
nsh> mount -t vfat /dev/sda /mnt/sda
nsh> ls /mnt/lfs
/mnt/lfs:
 .
 ..
nsh> ls /mnt/sda
/mnt/sda:
 afile
nsh> cat /mnt/sda/afile
This will stay on the USB drive
nsh> 
```

```
❯ telnet 192.168.86.249
Trying 192.168.86.249...
Connected to 192.168.86.249.
Escape character is '^]'.

NuttShell (NSH) NuttX-10.0.1
nsh> ps
  PID PRI POLICY   TYPE    NPX STATE    EVENT     SIGMASK   STACK   USED  FILLED    CPU COMMAND
    0   0 FIFO     Kthread N-- Ready              00000000 001024 000472  46.0%  100.0% Idle Task
    1 224 RR       Kthread --- Waiting  Signal    00000000 002036 000456  22.3%    0.0% hpwork
    2 100 RR       Kthread --- Waiting  Signal    00000000 002036 000680  33.3%    0.0% lpwork
    3 100 RR       Task    --- Waiting  Semaphore 00000000 004084 001304  31.9%    0.0% init
    4 100 RR       Kthread --- Waiting  Semaphore 00000000 002036 000752  36.9%    0.0% usbhost
    5  50 RR       Kthread --- Waiting  Signal    00000000 004076 000472  11.5%    0.0% USB Monitor
    6 100 RR       Task    --- Waiting  Semaphore 00000000 002012 000648  32.2%    0.0% Telnet daemon 0x38005600
    9 100 RR       Kthread --- Waiting  Semaphore 00000000 001004 000448  44.6%    0.0% telnet_io
   10 100 RR       Task    --- Running            00000000 002028 001328  65.4%    0.0% Telnet session
nsh> 
```