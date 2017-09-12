README
^^^^^^

README for NuttX port to the LC823450XGEVK board.

The board information is available at

  http://www.onsemi.com/PowerSolutions/evalBoard.do?id=LC823450XGEVK

LC823450 related documents are available at

  http://www.onsemi.com/PowerSolutions/supportDoc.do?type=AppNotes&rpn=LC823450

This port is intended to test LC823450 features including SMP.
Supported peripherals:
UART, TIMER, RTC, GPIO, DMA, I2C, SPI, LCD, eMMC, USB, WDT, ADC.

Settings
^^^^^^^^

1. Currently only SRAM boot via ICE is supported.
2. If SWD connection is lost, please specify lower adaptor clock.
3. Both CPUs are running at 160MHz.
4. Internal SRAMs (seg0 to seg5) are used.
5. Serial console can be used via external USB-UART (115200/8/N/1).
6. Interrupt handlers except for inter-cpu are handled on CPU0.

SMP related Status
^^^^^^^^^^^^^^^^^^

Currently SMP feature works on the board but might not be stable.
In addition, console output might be corrupted if the both CPUs
output into the console because UART operates in FIFO mode.

1. "nsh> smp" works but the result will be corrupted.
2. "nsh> ostest" works but might cause a deadlock or assertion.

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


TODO
^^^^

The following features will be supported.
IPL2 (eMMC boot), Audio, etc.
