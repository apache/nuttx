==============
ESP32S3-DevKit
==============

The `ESP32S3 DevKit <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html>`_ is a development board for the ESP32-S3 SoC from Espressif, based on a ESP32-S3-WROOM-1 module.

.. list-table::
   :align: center

   * - .. figure:: esp32-s3-devkitc-1.png
          :align: center

Features
========

  - ESP32-S3-WROOM-1 Module
  - USB-to-UART bridge via micro USB port
  - Power LED
  - EN and BOOT buttons (BOOT accessible to user)
  - SPI FLASH (size varies according to model

Serial Console
==============

UART0 is, by default, the serial console.  It connects to the on-board
CP2102 converter and is available on the USB connector USB CON8 (J1).

It will show up as /dev/ttyUSB[n] where [n] will probably be 0.

Buttons and LEDs
================

Board Buttons
-------------

There are two buttons labeled Boot and EN.  The EN button is not available
to software.  It pulls the chip enable line that doubles as a reset line.

The BOOT button is connected to IO0.  On reset it is used as a strapping
pin to determine whether the chip boots normally or into the serial
bootloader.  After reset, however, the BOOT button can be used for software
input.

Board LEDs
----------

There are several on-board LEDs for that indicate the presence of power
and USB activity.  None of these are available for use by software.
Another WS2812 LED is connected to GPIO48 or GPIO38 depending on the boards
version.

.. note:: Both the initial and v1.1 versions of ESP32-S3-DevKitC-1 are
   available on the market. The main difference lies in the GPIO assignment
   for the RGB LED: the initial version (1.0) uses GPIO48, whereas v1.1 uses
   GPIO38. The initial version is selected by default, but one can select
   ``CONFIG_ESP32S3_DEVKITC_1_V11`` through ``make menuconfig``.

I2S
===

ESP32-S3 has two I2S peripherals accessible using either the generic I2S audio
driver or a specific audio codec driver
(`CS4344 <https://www.cirrus.com/products/cs4344-45-48/>`__ bindings are
available at the moment). The generic I2S audio driver enables the use of both
the receiver module (RX) and the transmitter module (TX) without using any
specific codec. Also, it's possible to use the I2S character device driver
to bypass the audio subsystem and write directly to the I2S peripheral.

The following configurations use the I2S peripheral::
  * :ref:`platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:audio`
  * :ref:`platforms/xtensa/esp32s3/boards/esp32s3-devkit/index:nxlooper`

Configurations
==============

All of the configurations presented below can be tested by running the following commands::

    $ ./tools/configure.sh esp32s3-devkit:<config_name>
    $ make flash ESPTOOL_PORT=/dev/ttyUSB0 -j

Where <config_name> is the name of board configuration you want to use, i.e.: nsh, buttons, wifi...
Then use a serial console terminal like ``picocom`` configured to 115200 8N1.

audio
-----

This configuration uses the I2S0 peripheral and an externally connected audio
codec to play an audio file streamed over an HTTP connection while connected
to a Wi-Fi network.

**Audio Codec Setup**

The CS4344 audio codec is connected to the following pins:

============ ========== ============================================
ESP32-S3 Pin CS4344 Pin Description
============ ========== ============================================
5            MCLK       Master Clock
16           SCLK       Serial Clock
7            LRCK       Left Right Clock (Word Select)
6            SDIN       Serial Data In on CS4344. (DOUT on ESP32-S3)
============ ========== ============================================

**Simple HTTP server**

Prepare a PCM-encoded (`.wav`) audio file with 16 or 24 bits/sample (sampled at
16~48kHz). This file must be placed into a folder in a computer that could
be accessed on the same Wi-Fi network the ESP32 will be connecting to.

Python provides a simple HTTP server. ``cd`` to the audio file folder on the
PC and run::

    $ python3 -m http.server
    Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/)

Look for your PC IP address and test playing the prepared audio on your
browser:

.. figure:: esp32-audio-config-file.png
          :align: center

After successfully built and flashed, connect the board to the Wi-Fi network::

    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

Once connected, open NuttX's player and play the file according to the filename
and the IP address of the HTTP server::

    nsh> nxplayer
    nxplayer> play http://192.168.1.239:8000/tones.wav

buttons
-------

This configuration shows the use of the buttons subsystem. It can be used by executing
the ``buttons`` application and pressing on any of the available board buttons::

    nsh> buttons
    buttons_main: Starting the button_daemon
    buttons_main: button_daemon started
    button_daemon: Running
    button_daemon: Opening /dev/buttons
    button_daemon: Supported BUTTONs 0x01
    nsh> Sample = 1
    Sample = 0

capture
--------

The capture configuration enables the capture driver and the capture example, allowing
the user to measure duty cycle and frequency of a signal. Default pin is GPIO 12 with
an internal pull-up resistor enabled. When connecting a 50 Hz pulse with 50% duty cycle,
the following output is expected:

nsh> cap
cap_main: Hardware initialized. Opening the capture device: /dev/capture0
cap_main: Number of samples: 0
pwm duty cycle: 50 % 
pwm frequence: 50 Hz 
pwm duty cycle: 50 % 
pwm frequence: 50 Hz 

coremark
--------

This configuration sets the CoreMark benchmark up for running on the maximum
number of cores for this system. It also enables some optimization flags and
disables the NuttShell to get the best possible score.

.. note:: As the NSH is disabled, the application will start as soon as the
  system is turned on.

cxx
---

Development environment ready for C++ applications. You can check if the setup
was successful by running ``cxxtest``::

    nsh> cxxtest
    Test ofstream ================================
    printf: Starting test_ostream
    printf: Successfully opened /dev/console
    cout: Successfully opened /dev/console
    Writing this to /dev/console
    Test iostream ================================
    Hello, this is only a test
    Print an int: 190
    Print a char: d
    Test std::vector =============================
    v1=1 2 3
    Hello World Good Luck
    Test std::map ================================
    Test C++17 features ==========================
    File /proc/meminfo exists!
    Invalid file! /invalid
    File /proc/version exists!

gpio
----

This is a test for the GPIO driver. Three GPIOS are defined: 1) GPIO15 is
set as an output, 2) GPIO18 as input and, 3) GPIO21 as an input triggered
by a rising edge.

This example also builds the ``EXAMPLES_GPIO`` application from the
``nuttx-apps``.

To write to the GPIO (GPIO 15, as defined by the board implementation)::

    nsh> gpio -o 1 /dev/gpio0
    nsh> gpio -o 0 /dev/gpio0

To read from the GPIO (GPIO 18, as defined by the board implementation)::

    nsh> gpio /dev/gpio1
    Driver: /dev/gpio1
      Input pin:     Value=1

Finally, we can use the interrupt pin (GPIO21) to send a signal when the
interrupt fires::

    nsh> gpio -w 14 /dev/gpio2
    Driver: /dev/gpio2
      Interrupt pin: Value=0
      Verify:        Value=1

The pin is configured to trigger an interrupt on the rising edge, so after
issuing the above command, connect it to 3.3V.

i2c
---

This configuration can be used to scan and manipulate I2C devices.
You can scan for all I2C devices using the following command::

    nsh> i2c dev 0x00 0x7f

knsh
----

This is identical to the nsh configuration except that (1) NuttX
is built as PROTECTED mode, monolithic module and the user applications
are built separately and, as a consequence, (2) some features that are
only available in the FLAT build are disabled.

Protected Mode support for ESP32-S3 relies on the World Controller (WC)
and Permission Control (PMS) peripherals for implementing isolation
between Kernel and Userspace.

By working together with the MMU and Static MPUs of the ESP32-S3, the WC/PMS
is able to restrict the application access to peripherals, on-chip
memories (Internal ROM and Internal SRAM) and off-chip memories (External
Flash and PSRAM).

.. warning:: The World Controller and Permission Control **do not** prevent
  the application from accessing CPU System Registers.

mcuboot_nsh
-----------

This configuration is the same as the ``nsh`` configuration, but it generates the application
image in a format that can be used by MCUboot. It also makes the ``make bootloader`` command to
build the MCUboot bootloader image using the Espressif HAL.

nsh
---

Basic NuttShell configuration (console enabled in UART0, exposed via
USB connection by means of CP2102 converter, at 115200 bps).

nxlooper
--------

This configuration uses the I2S1 peripheral as an I2S receiver and the I2S0
peripheral as an I2S transmitter. The idea is to capture an I2S data frame
using an I2S peripheral and reproduce the captured data on the other.

**Receiving data on I2S1**

The I2S1 will act as a receiver (in slave mode, i.e., waiting for the BCLK
and WS signals from the transmitter), capturing data from DIN, which
needs to be connected to an external source as follows:

============ ========== =========================================
ESP32-S3 Pin Signal Pin Description
============ ========== =========================================
18           BCLK       Bit Clock (SCLK)
17           WS         Word Select (LRCLK)
15           DIN        Data IN
============ ========== =========================================

**Transmitting data on I2S0**

The I2S0 will act as a transmitter (in master mode, i.e., providing the
BCLK and WS signals), replicating the data captured on I2S1.
The pinout for the transmitter is as follows:

========== ========== =========================================
ESP32 Pin  Signal Pin Description
========== ========== =========================================
5          MCLK       Master Clock
16         BCLK       Bit Clock (SCLK)
7          WS         Word Select (LRCLK)
6          DOUT       Data Out
========== ========== =========================================

.. note:: The audio codec CS4344 can be connected to the transmitter pins
  to reproduce the captured data if the receiver's source is a PCM-encoded
  audio data.

**nxlooper**

The ``nxlooper`` application captures data from the audio device with input
capabilities (the I2S1 in this example) and forwards the audio data frame to
the audio device with output capabilities (the I2S0 in this example).

After successfully built and flashed, run on the boards' terminal::

  nsh> nxlooper
  nxlooper> loopback

.. note:: ``loopback`` command default arguments for the channel configuration,
  data width and sample rate are, respectively, 2 channels,
  16 bits/sample and 48KHz. These arguments can be supplied to select
  different audio formats, for instance::

    nxlooper> loopback 2 16 44100

oneshot
-------

This config demonstrate the use of oneshot timers present on the ESP32-S3.
To test it, just run the ``oneshot`` example::

    nsh> oneshot
    Opening /dev/oneshot
    Maximum delay is 4294967295999999
    Starting oneshot timer with delay 2000000 microseconds
    Waiting...
    Finished

pm
-------

This config demonstrate the use of power management present on the ESP32-S3.
You can use the ``pmconfig`` command to test the power management.
Enables PM support. You can define standby mode and sleep mode delay time::

    $ make menuconfig
    -> Board Selection
        -> (15) PM_STANDBY delay (seconds)
           (0)  PM_STANDBY delay (nanoseconds)
           (20) PM_SLEEP delay (seconds)
           (0)  PM_SLEEP delay (nanoseconds)

Before switching PM status, you need to query the current PM status::

    nsh> pmconfig
    Last state 0, Next state 0

    /proc/pm/state0:
    DOMAIN0           WAKE         SLEEP         TOTAL
    normal          0s 00%        0s 00%        0s 00%
    idle            0s 00%        0s 00%        0s 00%
    standby         0s 00%        0s 00%        0s 00%
    sleep           0s 00%        0s 00%        0s 00%

    /proc/pm/wakelock0:
    DOMAIN0      STATE     COUNT      TIME
    system       normal        2        1s
    system       idle          1        1s
    system       standby       1        1s
    system       sleep         1        1s

System switch to the PM idle mode, you need to enter::

    nsh> pmconfig relax normal
    nsh> pmconfig relax normal

System switch to the PM standby mode, you need to enter::

    nsh> pmconfig relax idle
    nsh> pmconfig relax normal
    nsh> pmconfig relax normal

System switch to the PM sleep mode, you need to enter::

    nsh> pmconfig relax standby
    nsh> pmconfig relax idle
    nsh> pmconfig relax normal
    nsh> pmconfig relax normal

Note: When normal mode COUNT is 0, it will switch to the next PM state where COUNT is not 0.

psram_quad
----------

This config tests the PSRAM driver over SPIRAM interface in quad mode.
You can use the mm command to test the PSRAM memory::

    nsh> mm
        mallinfo:
          Total space allocated from system = 8803232
          Number of non-inuse chunks        = 2
          Largest non-inuse chunk           = 8388592
          Total allocated space             = 9672
          Total non-inuse space             = 8793560
    (0)Allocating 5011 bytes

    ......

    (31)Releasing memory at 0x3fc8c088 (size=24 bytes)
        mallinfo:
          Total space allocated from system = 8803232
          Number of non-inuse chunks        = 2
          Largest non-inuse chunk           = 8388592
          Total allocated space             = 9672
          Total non-inuse space             = 8793560
    TEST COMPLETE

psram_octal
-----------

Similar to the ```psram_quad``` configuration but using the SPIRAM
interface in octal mode.

pwm
---

This configuration demonstrates the use of PWM through a LED connected to GPIO2.
To test it, just execute the ``pwm`` application::

    nsh> pwm
    pwm_main: starting output with frequency: 10000 duty: 00008000
    pwm_main: stopping output

qemu_debug
----------

A configuration tailored for the `Espressif fork of QEMU`_.

.. _Espressif fork of QEMU: https://github.com/espressif/qemu

random
------

This configuration shows the use of the ESP32-S3's True Random Number Generator with
entropy sourced from Wi-Fi and Bluetooth noise.
To test it, just run ``rand`` to get 32 randomly generated bytes::

    nsh> rand
    Reading 8 random numbers
    Random values (0x3ffe0b00):
    0000  98 b9 66 a2 a2 c0 a2 ae 09 70 93 d1 b5 91 86 c8  ..f......p......
    0010  8f 0e 0b 04 29 64 21 72 01 92 7c a2 27 60 6f 90  ....)d!r..|.'`o.

rmt
---

This configuration configures the transmitter and the receiver of the
Remote Control Transceiver (RMT) peripheral on the ESP32-S3 using GPIOs 48
(or 38, depending on the board version) and 2, respectively.
The RMT peripheral is better explained
`here <https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html>`__,
in the ESP-IDF documentation. The minimal data unit in the frame is called the
RMT symbol, which is represented by ``rmt_item32_t`` in the driver:

.. figure:: rmt_symbol.png
   :align: center

The example ``rmtchar`` can be used to test the RMT peripheral. Connecting
these pins externally to each other will make the transmitter send RMT items
and demonstrates the usage of the RMT peripheral::

    nsh> rmtchar

**WS2812 addressable RGB LEDs**

This same configuration enables the usage of the RMT peripheral and the example
``ws2812`` to drive addressable RGB LEDs::

    nsh> ws2812

Please note that this board contains an on-board WS2812 LED connected to GPIO48
(or GPIO38, depending on the board version) and, by default, this config
configures the RMT transmitter in the same pin.

rtc
---

This configuration demonstrates the use of the RTC driver through alarms.
You can set an alarm, check its progress and receive a notification after it expires::

    nsh> alarm 10
    alarm_daemon started
    alarm_daemon: Running
    Opening /dev/rtc0
    Alarm 0 set in 10 seconds
    nsh> alarm -r
    Opening /dev/rtc0
    Alarm 0 is active with 10 seconds to expiration
    nsh> alarm_daemon: alarm 0 received

smp
---

Another NSH configuration, similar to nsh, but also enables
SMP operation.  It differs from the nsh configuration only in these
additional settings:

SMP is enabled::

  CONFIG_SMP=y
  CONFIG_SMP_NCPUS=2
  CONFIG_SPINLOCK=y

The apps/testing/smp test is included::

  CONFIG_TESTING_SMP=y
  CONFIG_TESTING_SMP_NBARRIER_THREADS=8
  CONFIG_TESTING_SMP_PRIORITY=100
  CONFIG_TESTING_SMP_STACKSIZE=2048

spiflash
--------

This config tests the external SPI that comes with the ESP32-S3 module connected
through SPI1.

By default a SmartFS file system is selected.
Once booted you can use the following commands to mount the file system::

    nsh> mksmartfs /dev/smart0
    nsh> mount -t smartfs /dev/smart0 /mnt

Note that mksmartfs is only needed the first time.

sta_softap
----------

With this configuration you can run these commands to be able
to connect your smartphone or laptop to your board::

  nsh> ifup wlan1
  nsh> dhcpd_start wlan1
  nsh> wapi psk wlan1 mypasswd 3
  nsh> wapi essid wlan1 nuttxap 1

In this case, you are creating the access point ``nuttxapp`` in your board and to
connect to it on your smartphone you will be required to type the password ``mypasswd``
using WPA2.

.. tip:: Please refer to :ref:`ESP32 Wi-Fi SoftAP Mode <esp32_wi-fi_softap>`
  for more information.

The ``dhcpd_start`` is necessary to let your board to associate an IP to your smartphone.

tickless
--------

This configuration enables the support for tickless scheduler mode.

timer
-----

This config test the general use purpose timers. It includes the 4 timers,
adds driver support, registers the timers as devices and includes the timer
example.

To test it, just run the following::

  nsh> timer -d /dev/timerx

Where x in the timer instance.

toywasm
-------

This config is an example to use toywasm.

This example uses littlefs on the SPI flash to store wasm modules.

Note: This example assumes a board with 32MB flash. To use a smaller one,
tweak the --img-size option and CONFIG_ESP32S3_STORAGE_MTD_SIZE.

Note: To use flash larger than 4MB, you need to install a custom bootloader.
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/bootloader.html#spi-flash-configuration

1. Create a littlefs image which contains wasm modules.

   https://github.com/jrast/littlefs-python/blob/master/examples/mkfsimg.py
   is used in the following example::

      % python3 mkfsimg.py \
        --img-filename ..../littlefs.bin \
        --img-size 31981568 \
        --block-size 4096 \
        --prog-size 256 \
        --read-size 256 \
        --name-max 32 \
        --disk-version 2.0 \
        ..../wasm_module_dir

2. Build a NuttX binary as usual with this config.

3. Write the NuttX binary and the filesystem image to the board::

      % esptool.py \
        -c esp32s3 \
        -p /dev/tty.SLAB_USBtoUART \
        -b 921600 \
        write_flash \
        -fs detect \
        -fm dio \
        -ff 40m \
        0x10000 nuttx.bin \
        0x180000 ..../littlefs.bin

4. Mount the filesystem and run a wasm module on it::

      nsh> mount -t littlefs /dev/esp32s3flash /mnt
      nsh> toywasm --print-stats --wasi /mnt/....

twai
----

This configuration enables the support for the TWAI (Two-Wire Automotive Interface) driver.
You can test it by connecting TWAI RX and TWAI TX pins which are GPIO0 and GPIO2 by default
to a external transceiver or connecting TWAI RX to TWAI TX pin by enabling
the ``Device Drivers -> CAN Driver Support -> CAN loopback mode`` option and running the ``can`` example::

    nsh> can
    nmsgs: 0
    min ID: 1 max ID: 2047
    Bit timing:
      Baud: 1000000
      TSEG1: 15
      TSEG2: 4
        SJW: 3
      ID:    1 DLC: 1

usbnsh
------

Basic NuttShell configuration console enabled over USB Device (USB CDC/ACM).

Before using this configuration, please confirm that your computer detected
that USB JTAG/serial interface used to flash the board::

  usb 3-5.2.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
  usb 3-5.2.3: Product: USB JTAG/serial debug unit
  usb 3-5.2.3: Manufacturer: Espressif
  usb 3-5.2.3: SerialNumber: XX:XX:XX:XX:XX:XX
  cdc_acm 3-5.2.3:1.0: ttyACM0: USB ACM device

Then you can run the configuration and compilation procedure::

  $ ./tools/configure.sh esp32s3-devkit:usbnsh
  $ make flash ESPTOOL_PORT=/dev/ttyACM0 -j8

Then run the minicom configured to /dev/ttyACM0 115200 8n1 and
press <ENTER> three times to force the nsh to show up::

  NuttShell (NSH) NuttX-12.1.0
  nsh> ?
  help usage:  help [-v] [<cmd>]

      .         break     dd        exit      ls        ps        source    umount
      [         cat       df        false     mkdir     pwd       test      unset
      ?         cd        dmesg     free      mkrd      rm        time      uptime
      alias     cp        echo      help      mount     rmdir     true      usleep
      unalias   cmp       env       hexdump   mv        set       truncate  xd
      basename  dirname   exec      kill      printf    sleep     uname

  Builtin Apps:
      nsh  sh
  nsh> uname -a
  NuttX 12.1.0 38a73cd970 Jun 18 2023 16:58:46 xtensa esp32s3-devkit
  nsh>

wifi
----

Enables Wi-Fi support. You can define your credentials this way::

    $ make menuconfig
    -> Application Configuration
        -> Network Utilities
            -> Network initialization (NETUTILS_NETINIT [=y])
                -> WAPI Configuration

Or if you don't want to keep it saved in the firmware you can do it
at runtime::

    nsh> wapi psk wlan0 mypasswd 3
    nsh> wapi essid wlan0 myssid 1
    nsh> renew wlan0

.. tip:: Please refer to :ref:`ESP32 Wi-Fi Station Mode <esp32_wi-fi_sta>`
  for more information.

watchdog
--------

This config test the watchdog timers. It includes the 2 MWDTS,
adds driver support, registers the WDTs as devices and includes the watchdog
example.

To test it, just run the following::

  nsh> wdog -i /dev/watchdogx

Where x is the watchdog instance.

To test the XTWDT(/dev/watchdog3) an interrupt handler needs to be
implemented because XTWDT does not have system reset feature. To implement
an interrupt handler `WDIOC_CAPTURE` command can be used. When interrupt
rises, XTAL32K clock can be restored with `WDIOC_RSTCLK` command.
