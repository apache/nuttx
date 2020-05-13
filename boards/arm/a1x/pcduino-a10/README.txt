README
======

  This directory contains the port of NuttX to the pcDuino v1 board
  See http://www.pcduino.com/ for information about pcDuino Lite, v1,
  and v2.  These boards are based around the Allwinner A10 Cortex-A8 CPU.
  I have not compared these boards in detail, but I believe that the
  differences are cosmetic.  This port was developed on the v1 board, but
  the others may be compatible:

  pcDuino Lite (See http://www.pcduino.com/?page_id=1707)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 512B
    Onboard Storage      NO Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v1 (http://www.pcduino.com/?page_id=12)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
 *  DRAM                 1GB
 *  Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
    Extension Interface  2.54mm Headers
    Network interface    10/100Mbps RJ45 and USB WiFi extension (not included)
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  pcDuino v2 (http://www.pcduino.com/?page_id=1618)

    ITEMS                DETAILS
    -------------------- ---------------------------------------------------
    Items                Details
    CPU                  1GHz ARM Cortex A8
    GPU                  OpenGL ES2.0, OpenVG 1.1 Mali 400 core
    DRAM                 1GB
    Onboard Storage      2GB Flash, microSD card (TF) slot for up to 32GB
    Video Output         HDMI
 *  Extension Interface  Arduino Headers
 *  Network interface    10/100Mbps RJ45 and on-board WiFi module
    Power                5V, 2000mA
    Overall Size         125mm X 52mm

  Main features of the Allwinner A10
  (See http://www.allwinnertech.com/en/product/a10.html):

  CPU
    - ARM Cortex™-A8
    - 32KB I-Cache
    - 32KB D-Cache
    - 256KB L2 Cache

  GPU
    - ARM Mali-400

  Video
    - UHD 2160P video decoding
    - 3D video decoding
    - Support various video decoding formats, including VP8, AVS, H. 264
      MVC, VC-1, MPEG-1,2,4, etc
    - H.264 HP video encoding up to 1080p @ 30 fps or dual-channel 720p @ 30
      fps

  Display
    - Multi-channel HD display
    - Integrated HDMI 1.4
    - YPbPr, CVBS, VGA
    - Multiple LCD interfaces, including CPU, RGB, LVDS up to Full HD

  Memory
    - 32-bit DDR2/DDR3
    - Memory capacity up to 16G bits
    - SLC/MLC/TLC/DDR NAND
    - 8 flash chips, 64-bit ECC

        Memory capacity up to 64GB
        Support NAND of 5xnm, 4xnm, 3xnm, 2xnm, etc
        Support NAND of Samsung, Toshiba, Hynix, etc

  Boot Devices
    - NAND Flash
    - SPI NOR Flash
    - SD Card
    - USB

Contents
========

  - pcDuino v1 Connectors
  - Serial Console
  - LEDs
  - Buttons
  - JTAG
  - Booting NuttX from an SD card
  - Configurations

pcDuino v1 Connectors
=====================

  TOP
  ---
  - HDMI
  - RJ45
  - USB Host (2)

  - J11
     1. UART-Rx / GPIO0            UART2_RX
     2. UART-Tx / GPIO1            UART2_TX
     3. GPIO3 / GPIO2              GPIO2
     4. PWM0 / GPIO3               PWM0
     5. GPIO4                      GPIO3
     6. PWM1 / GPIO5               PWM1
     7. PWM2 /GPIO6                PWM2
     8. GPIO7                      GPIO4

  - J8
     1. GPIO8                      GPIO5
     2. PWM3 / GPIO9               PWM3
     3. SPI_CS / GPIO10 / PWM4     SPI0_CS
     4. SPI_MOSI / GPIO11 / PWM5   SPI0_MOSI
     5. SPI_MISO / GPIO12          SPI0_MISO
     6. SPI_CLK / GPIO13           SPI0_CLK
     7. Gnd
     8. ARef
     9. I2C-SDA                    TWI2_SDA
    10. I2C-SCK                    TWI2_SCK

  - J12
     1. ADC0
     2. ADC1
     3. ADC2
     4. ADC3
     5. ADC4
     6. ADC5

  - J9
     1. 5V
     2. Gnd
     3. Gnd
     4. 5V
     5. 3.3V
     6. Reset
     7. 5V
     8. NC

  - J5 Debug Port
     1. Rx                         UART0-RX
     2. Gnd                        GND
     3. Tx                         UART0-TX

  - J6 SPI2
     1. SPI2_MISO
     2. DC_5V
     3. SPI2_CLK
     4. SPI2_MOSI
     5. RESET#
     6. GND

  - J7 SPI0
     1. SPI0_MISO
     2. DC_5V
     3. SPI0_CLK
     4. SPI0_MOSI
     5. RESET#
     6. GND

  - J10
     1. GPIO6
     2. GPIO8
     3. GPIO7
     4. GPIO9

  Bottom
  ------
  - USB OTG
  - DC Power IN (USB)
  - microSD card slot

Serial Console
==============

  1. UART0 is available on J5 Debug Port.

     J15 Pin 1 Rx                UART0-RX  UART0_RX/IR1_RX/PB23
     J15 Pin 3 Tx                UART0-TX  UART0_TX/IR1_TX/PB22

  2. UART2 is available on J11

     J11 Pin1  UART-Rx / GPIO0   UART2_RX  EINT31/SPI1_MISO/UART2_RX/PI19
     J11 Pin2  UART-Tx / GPIO1   UART2_TX  EINT30/SPI1_MOSI/UART2_TX/PI18

  By default, the serial console will be provided on UART0 in all of these
  configurations.

LEDs
====

  The pcDuino v1 has four green LEDs; three can be controlled from software.
  Two are tied to ground and, hence, illuminated by driving the output pins
  to a high value:

    1. LED1 SPI0_CLK  SPI0_CLK/UART5_RX/EINT23/PI11
    2. LED5 IPSOUT    From the PMU (not controllable by software)

  And two are pull high and, hence, illuminated by grounding the output:

    3. LED3 RX_LED    LCD1_D16/ATAD12/KP_IN6/SMC_DET/EINT16/CSI1_D16/PH16
    4. LED4 TX_LED    LCD1_D15/ATAD11/KP_IN5/SMC_VPPPP/EINT15/CSI1_D15/PH15

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL            Meaning                      LED state
                                               LED1 LED3 LED4
    ----------------- -----------------------  ---- ---- ------------
    LED_STARTED       NuttX has been started   ON   OFF  OFF
    LED_HEAPALLOCATE  Heap has been allocated  OFF  ON   OFF
    LED_IRQSENABLED   Interrupts enabled       ON   ON   OFF
    LED_STACKCREATED  Idle stack created       ON   ON   OFF
    LED_INIRQ         In an interrupt          N/C  N/C  Soft glow
    LED_SIGNAL        In a signal handler      N/C  N/C  Soft glow
    LED_ASSERTION     An assertion failed      N/C  N/C  Soft glow
    LED_PANIC         The system has crashed   N/C  N/C  2Hz Flashing
    LED_IDLE          MCU is is sleep mode         Not used

  After booting, LED1 and 3 are not longer used by the system and can be used for
  other purposes by the application (Of course, all LEDs are available to the
  application if CONFIG_ARCH_LEDS is not defined.

Buttons
=======

  There are a total of five switches on-board.  All pulled high and, hence,
  will be sensed as low when closed.

    SW1 Reset     (not available to software)
    SW2 UBOOT     UBOOT_SEL (?)
    SW3 Key_Back  LCD1_D17/ATAD13/KP_IN7/SMC_VCCEN/EINT17/CSI1_D17/PH17
    SW4 Key_Home  LCD1_D18/ATAD14/KP_OUT0/SMC_SLK/EINT18/CSI1_D18/PH18
    SW5 Key_Menu  LCD1_D19/ATAD15/KP_OUT1/SMC_SDA/EINT19/CSI1_D19/PH19

JTAG
====

  A. I didn't get success testing J-Link with pcDuino, it is reading TDI
     always as 1.

     I think the main problem is because pcDuino JTAG doesn't have RESET
     (no trst or srst). I tried to connect the JTAG reset to Power_Reset
     of pcDuino, but it didn't work.

  B. Notice that the OlinuxIno JTAG does have a reset line called RESET_N.
     But it is nothing special.  It just connects to the RESET# pin C14 on
     the A10.  The pcDuino also brings out the RESET# on several connectors.

     So it seems like you could get the reset line if you need it, just not
     from the set of JTAG pads.

  A. I discovered the issue in the JTAG, it was not working because
     JTAG_SEL was not tied to GND.

    I compared the Olimex schematic with pcDuino and noticed there is a
    R64 resister that is not placed in the board.

    It was a little bit difficult to find this resistor, because it is
    "hidden" among the capacitors in the bottom of the board.

    After short circuiting the resistor PADs the JTAG started to work,
    well, JLinkExe now recognize it, but OpenOCD is not working yet.

Booting NuttX from an SD card
=============================

  These are the steps to get U-Boot booting from SD Card:

    1. Get the U-Boot sources for the pcDuino

       $ git clone https://github.com/yuq/u-boot-sunxi.git

    2. Build U-Boot.  We really only want the SPL program; this builds
       the whole thing:

       $ cd u-boot-sunxi
       $ make pcduino CROSS_COMPILE=arm-none-eabi-

       At the conclusion of a success bin, you will find the u-boot binary
       at ./u-boot.bin and the SPL binary at ./spl/sunxi-spl.bin

       NOTES:
       a. You may need to use a different tool prefix for the CROSS_COMPILE=
          value, depending upon what toolchain you have installed and upon
          which platform your are working.
       b. When I try this on Cygwin, I get a make failure that is, apparently,
          due to some script incompatibility.

    3. Insert a FLASH stick.  Use dmesg to get the name of the new USB
       device.  Make sure that it is not mounted, then (assuming that the
       USB device is /dev/sdb):

       $ sudo dd if=./spl/sunxi-spl.bin of=/dev/sdb bs=1024 seek=8
       $ sudo dd if=nuttx.bin of=/dev/sdb bs=1024 seek=32

    4. Remove the FLASH stick from the host pc.  Insert into the pcDuino
       microSD slot.  Reset the pcDuino and NuttX should be running.

  Reference: https://www.olimex.com/wiki/Bare_Metal_programming_A13#Stand_alone_program_running_with_uboot

Configurations
==============

  Information Common to All Configurations
  ----------------------------------------
  Each pcDuino configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh [OPTIONS] pcduino-a10:<subdir>

  Where [OPTIONS] include -l to configure for a Linux host platform and
  -c means to configure for a Windows Cygwin host platform.  -h will give
  you the list of all options.

  Before building, make sure the PATH environment variable includes the
  correct path to the directory than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       see additional README.txt files in the NuttX tools repository.

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART0.

  3. All of these configurations use the Code Sourcery for Windows toolchain
     (unless stated otherwise in the description of the configuration).  That
     toolchain selection can easily be reconfigured using 'make menuconfig'.
     Here are the relevant current settings:

     Build Setup:
       CONFIG_HOST_WINDOWS=y                   : Microsoft Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7A_TOOLCHAIN_GNU_EABIW=y     : GNU EABI toolchain for Windows

  Configuration Sub-directories
  -----------------------------

  nsh:

    This configuration directory provide the NuttShell (NSH).  There are

    STATUS:
      This configuration builds and runs, but only if the patch at
      nuttx/boards/arm/a1x/pcduino-a10/nsh/pcduino-140107.patch is applied.  This patchfile
      contains some fixes that are as-of-yet not well understood and so cannot be checked
      in.  Below is a summary of the kludges currently in this patch file:

      a) nuttx/arch/arm/src/armv7-a/arm_head.S: Initializes the MMU so that A10
         peripherals can be accessed very early.  This is not normally necessary, but
         is required because of certain debug statements that seem to be necessary
         in a1x_boot.c (see the next item).

      b) nuttx/arch/arm/src/a1x/a1x_boot.c:  This file contains several arbitrary
         statements that just output debug information.  Some of these can be removed,
         but if you remove all of the debug output, the pcDuino will not boot.  No
         idea yet why.

      c) nuttx/arch/arm/src/armv7-a/arm_mmu.c:  After setting a page table entry
         for the MMU, the MMU's TLBs are flushed for that memory region.  That
         flushing must currently be commented out.  Why?  I am not sure, but I
         think that this is because TLBs are being flushed why they are in use.  For
         the pcDuino, we are executing out of SDRAM so when the TLBs for the SDRAM
         region are invalidated that cause a crash.  That has not been proven,
         however.
