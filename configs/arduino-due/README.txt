README
^^^^^^

  This README discusses issues unique to NuttX configurations for the
  Arduino DUE board featuring the Atmel ATSAM3X8E MCU running at 84
  MHz.

Contents
^^^^^^^^

  - PIO Pin Usage
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Buttons and LEDs
  - Serial Consoles
  - SAM4S Xplained-specific Configuration Options
  - Configurations

PIO Pin Usage
^^^^^^^^^^^^^

  PORTA                          PORTB                          PORTC
  ------------------------------ ------------------------------ --------------------------------
  PIO   SIGNAL     CONN PIN      PIO   SIGNAL       CONN PIN    PIO   SIGNAL      CONN PIN
  ----- ---------- ---- -------- ----- ------------ ---- ------ ----- ----------- ---- ---------
  PA0   CANTX0     ADCH 8        PB0   ETX_CLK      ETH  1      PC0   ERASE       N/A
  PA1   CANRX0     ACDH 7        PB1   ETX_EN       ETH  3      PC1   PIN33       XIO  14
  PA2   AD7        ADCL 7        PB2   ETXD0        ETH  5      PC2   PIN34       XIO  15
  PA3   AD6        ADCL 6        PB3   ETXD1        ETH  7      PC3   PIN35       XIO  16
  PA4   AD5        ADCL 5        PB4   ERX_DV       ETH  10     PC4   PIN36       XIO  17
  PA5   EEXTINT    ETH  8        PB5   ERXD0        ETH  9      PC5   PIN37       XIO  18
  PA6   AD4        ADCL 4        PB6   ERXD1        ETH  11     PC6   PIN38       XIO  19
  PA7   PIN31      XIO  12       PB7   ERX_ER       ETH  13     PC7   PIN39       XIO  20
  PA8   [U]RX      PWML 1        PB8   EMDC         ETH  14     PC8   PIN40       XIO  21
  PA9   [U]TX      PWML 2        PB9   EMDIO        ETH  12     PC9   PIN41       XIO  22
  PA10  RXD2       COMM 6        PB10  UOTGVBOF     Vbus power  PC10  N/C         N/A
  PA11  TXD2       COMM 5        PB11  UOTGID       USB1 4      PC11  N/C         N/A
  PA12  RXD1       COMM 4        PB12  SDA0-3       COMM 7      PC12  PIN51       XIO  32
  PA13  TXD1       COMM 3        PB13  SCL0-3       COMM 8      PC13  PIN50       XIO  31
  PA14  PIN23      XIO  4        PB14  CANTX1/IO    XIO  34     PC14  PIN49       XIO  30
  PA15  PIN24      XIO  5        PB15  DAC0(CANRX1) ADCH 5      PC15  PIN48       XIO  29
  PA16  AD0        ADCL 0        PB16  DAC1         ADCH 6      PC16  PIN47       XIO  28
  PA17  SDA1       PWMH 9        PB17  AD8          ADCH 1      PC17  PIN46       XIO  27
  PA18  SCL1       PWMH 10       PB18  AD9          ADCH 2      PC18  PIN45       XIO  26
  PA19  PIN42      XIO  23       PB19  AD10         ADCH 3      PC19  PIN44       XIO  25
  PA20  PIN43      XIO  24       PB20  AD11(TXD3)   ADCH 4      PC20  N/C         N/A
  PA21  TXL        TX YELLOW LED PB21  AD14(RXD3)   XIO  33     PC21  PWM9        PWMH 2
  PA22  AD3        ADCL 3        PB22  N/C          N/A         PC22  PWM8        PWMH 1
  PA23  AD2        ADCL 2        PB23  SS3          ???         PC23  PWM7        PWML 8
  PA24  AD1        ADCL 1        PB24  N/C          N/A         PC24  PWM6        PWML 7
  PA25  MISO       SPI  1        PB25  PWM2         PWML 3      PC25  PWM5        PWML 6
  PA26  MOSI       SPI  4        PB26  PIN22        ???         PC26  SS1/PWM4    ??? (there are two)
  PA27  SPCK       SPI  3        PB27  PWM13        PWMH 6      PC27  N/C         N/A
  PA28  SS0/PWM10  (ETH)         PB28  JTAG_TCK     JTAG 4      PC28  PWM3        PWML 4
  PA29  SS1/PWM4   (SD)          PB29  JTAG_TDI     JTAG 8      PC29  SS0/PWM10   ??? (there are two)
  PA30  N/A         N/A          PB30  JTAG_TDO     JTAG 6      PC30  RXL         RX YELLOW LED
  PA31  N/A         N/A          PB31  JTAG_TMS     JTAG 2      PC31  N/A         N/A
  ----- ---------- ---- -------- ----- ------------ ---- ------ ----- ----------- ---- ---------

  PORTA                          PORTB                          PORTC
  ------------------------------ ------------------------------ --------------------------------
  PIO   SIGNAL     CONN PIN      PIO   SIGNAL       CONN PIN    PIO   SIGNAL      CONN PIN
  ----- ---------- ---- -------- ----- ------------ ---- ------ ----- ----------- ---- ---------
  PA0   PIN25       XIO  6       PE0   N/A          N/A         PF0   N/A         N/A
  PD1   PIN26       XIO  7       PE1   N/A          N/A         PF1   N/A         N/A
  PD2   PIN27       XIO  8       PE2   N/A          N/A         PF2   N/A         N/A
  PD3   PIN28       XIO  9       PE3   N/A          N/A         PF3   N/A         N/A
  PD4   TXD0        COMM 1       PE4   N/A          N/A         PF4   N/A         N/A
  PD5   RXD0        COMM 2       PE5   N/A          N/A         PF5   N/A         N/A
  PD6   PIN29       XIO  10      PE6   N/A          N/A         PF6   N/A         N/A
  PD7   PWM11       PWMH 4       PE7   N/A          N/A         PF7   N/A         N/A
  PD8   PWM12       PWMH 5       PE8   N/A          N/A         PF8   N/A         N/A
  PD9   PIN30       XIO  11      PE9   N/A          N/A         PF9   N/A         N/A
  PD10  PIN32       XIO  13      PE10  N/A          N/A         PF10  N/A         N/A
  PD11  N/A         N/A          PE11  N/A          N/A         PF11  N/A         N/A
  PD12  N/A         N/A          PE12  N/A          N/A         PF12  N/A         N/A
  PD13  N/A         N/A          PE13  N/A          N/A         PF13  N/A         N/A
  PD14  N/A         N/A          PE14  N/A          N/A         PF14  N/A         N/A
  PD15  N/A         N/A          PE15  N/A          N/A         PF15  N/A         N/A
  PD16  N/A         N/A          PE16  N/A          N/A         PF16  N/A         N/A
  PD17  N/A         N/A          PE17  N/A          N/A         PF17  N/A         N/A
  PD18  N/A         N/A          PE18  N/A          N/A         PF18  N/A         N/A
  PD19  N/A         N/A          PE19  N/A          N/A         PF19  N/A         N/A
  PD20  N/A         N/A          PE20  N/A          N/A         PF20  N/A         N/A
  PD21  N/A         N/A          PE21  N/A          N/A         PF21  N/A         N/A
  PD22  N/A         N/A          PE22  N/A          N/A         PF22  N/A         N/A
  PD23  N/A         N/A          PE23  N/A          N/A         PF23  N/A         N/A
  PD24  N/A         N/A          PE24  N/A          N/A         PF24  N/A         N/A
  PD25  N/A         N/A          PE25  N/A          N/A         PF25  N/A         N/A
  PD26  N/A         N/A          PE26  N/A          N/A         PF26  N/A         N/A
  PD27  N/A         N/A          PE27  N/A          N/A         PF27  N/A         N/A
  PD28  N/A         N/A          PE28  N/A          N/A         PF28  N/A         N/A
  PD29  N/A         N/A          PE29  N/A          N/A         PF29  N/A         N/A
  PD30  N/A         N/A          PE30  N/A          N/A         PF30  N/A         N/A
  PD31  N/A5        N/A          PE31  N/A          N/A         PF31  N/A         N/A
  ----- ---------- ---- -------- ----- ------------ ---- ------ ----- ----------- ---- ---------

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain, ok
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_ARMV7M_TOOLCHAIN_ATOLLIC=y        : Atollic toolchain for Windos
    CONFIG_ARMV7M_TOOLCHAIN_DEVKITARM=y      : devkitARM under Windows
    CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIL=y      : Generic GCC ARM EABI toolchain for Linux
    CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y      : Generic GCC ARM EABI toolchain for Windows

  If you are not using CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT, then you may also
  have to modify the PATH in the setenv.h file if your make cannot find the
  tools.

  NOTE about Windows native toolchains
  ------------------------------------

  The CodeSourcery (for Windows), Atollic, and devkitARM toolchains are
  Windows native toolchains.  The CodeSourcery (for Linux), NuttX buildroot,
  and, perhaps, the generic GCC toolchains are Cygwin and/or Linux native
  toolchains. There are several limitations to using a Windows based
  toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project (There is a simple RIDE project
  in the RIDE subdirectory).

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/sam34,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/sam34/sam_vectors.S.  You may need to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.shsam4s-xplained/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

NXFLAT Toolchain
^^^^^^^^^^^^^^^^

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).

  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1768/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

Buttons and LEDs
^^^^^^^^^^^^^^^^

  Buttons
  -------
  There are no buttons on the Arduino Due board.

  LEDs
  ----
  There are two user-controllable LEDs on board the Arduino Due board:

      LED              GPIO
      ---------------- -----
      TX  Yellow LED   PA21
      RX  Yellow LED   PC30

  Both are pulled high and can be illuminated by driving the corresponding
  GPIO output to low.

  These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
  defined.  In that case, the usage by the board port is defined in
  include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
  events as follows:

    SYMBOL                Meaning                     LED state
                                                    RX       TX
    -------------------  -----------------------  -------- --------
    LED_STARTED          NuttX has been started     OFF      OFF
    LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
    LED_IRQSENABLED      Interrupts enabled         OFF      OFF
    LED_STACKCREATED     Idle stack created         ON       OFF
    LED_INIRQ            In an interrupt              No change
    LED_SIGNAL           In a signal handler          No change
    LED_ASSERTION        An assertion failed          No change
    LED_PANIC            The system has crashed     OFF      Blinking
    LED_IDLE             MCU is is sleep mode         Not used

  Thus if RX is statically on, NuttX has successfully booted and is,
  apparently, running normmally.  If TX is flashing at approximately
  2Hz, then a fatal error has been detected and the system has halted.


Serial Consoles
^^^^^^^^^^^^^^^

  Any of UART and USART0-3 may be used as a serial console.  By default,
  the UART is used as the serial console in all configurations.  But that is
  easily changed by modifying the configuration as described under
  "Configurations" below.

    ------------------------------
    PIO   SIGNAL     CONN PIN
    ----- ---------- ---- --------
    PA8   [U]RX      PWML 1
    PA9   [U]TX      PWML 2
    PD4   TXD0       COMM 1
    PD5   RXD0       COMM 2
    PA12  RXD1       COMM 4
    PA13  TXD1       COMM 3
    PA10  RXD2       COMM 6
    PA11  TXD2       COMM 5
    PB20  AD11(TXD3) ADCH 4
  PB21  AD14(RXD3) XIO  33

  The outputs from these pins is 3.3V.  You will need to connect RS232
  transceiver to get the signals to RS232 levels (or connect the pins to the
  USB virual COM port.

Arduino DUE-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
  be set to:

    CONFIG_ARCH=arm

  CONFIG_ARCH_family - For use in C code:

    CONFIG_ARCH_ARM=y

  CONFIG_ARCH_architecture - For use in C code:

    CONFIG_ARCH_CORTEXM3=y

  CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

    CONFIG_ARCH_CHIP="sam34"

  CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
  chip:

    CONFIG_ARCH_CHIP_SAM34
    CONFIG_ARCH_CHIP_SAM3X
    CONFIG_ARCH_CHIP_ATSAM3X8E

  CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
  hence, the board that supports the particular chip or SoC.

    CONFIG_ARCH_BOARD=arduino-due (for the Arduino Due development board)

  CONFIG_ARCH_BOARD_name - For use in C code

    CONFIG_ARCH_BOARD_ARDUINO_DUE=y

  CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
  of delay loops

  CONFIG_ENDIAN_BIG - define if big endian (default is little
  endian)

  CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

    CONFIG_DRAM_SIZE=0x00008000 (32Kb)

  CONFIG_DRAM_START - The start address of installed DRAM

    CONFIG_DRAM_START=0x20000000

  CONFIG_ARCH_IRQPRIO - The SAM3UF103Z supports interrupt prioritization

    CONFIG_ARCH_IRQPRIO=y

  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
  have LEDs

  CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
  stack. If defined, this symbol is the size of the interrupt
  stack in bytes.  If not defined, the user task stacks will be
  used during interrupt handling.

  CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

  CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
  cause a 100 second delay during boot-up.  This 100 second delay
  serves no purpose other than it allows you to calibratre
  CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
  the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
  the delay actually is 100 seconds.

  Individual subsystems can be enabled:

    CONFIG_SAM34_RTC         - Real Time Clock
    CONFIG_SAM34_RTT         - Real Time Timer
    CONFIG_SAM34_WDT         - Watchdog Timer
    CONFIG_SAM34_UART0       - UART 0
    CONFIG_SAM34_SMC         - Static Memory Controller
    CONFIG_SAM34_SDRAMC      - SDRAM Controller
    CONFIG_SAM34_USART0      - USART 0
    CONFIG_SAM34_USART1      - USART 1
    CONFIG_SAM34_USART2      - USART 2
    CONFIG_SAM34_USART3      - USART 3
    CONFIG_SAM34_HSMCI       - High Speed Multimedia Card Interface
    CONFIG_SAM34_TWI0        - Two-Wire Interface 0 (master/slave)
    CONFIG_SAM34_TWI1        - Two-Wire Interface 1 (master/slave)
    CONFIG_SAM34_SPI0        - Serial Peripheral Interface 0
    CONFIG_SAM34_SPI1        - Serial Peripheral Interface 1
    CONFIG_SAM34_SSC         - Synchronous Serial Controller
    CONFIG_SAM34_TC0         - Timer Counter 0
    CONFIG_SAM34_TC1         - Timer Counter 1
    CONFIG_SAM34_TC2         - Timer Counter 2
    CONFIG_SAM34_TC3         - Timer Counter 3
    CONFIG_SAM34_TC4         - Timer Counter 4
    CONFIG_SAM34_TC5         - Timer Counter 5
    CONFIG_SAM34_TC6         - Timer Counter 6
    CONFIG_SAM34_TC7         - Timer Counter 7
    CONFIG_SAM34_TC8         - Timer Counter 8
    CONFIG_SAM34_PWM         - Pulse Width Modulation
    CONFIG_SAM34_ADC12B      - 12-bit Analog To Digital Converter
    CONFIG_SAM34_DACC        - Digital To Analog Converter
    CONFIG_SAM34_DMA         - DMA Controller
    CONFIG_SAM34_UOTGHS      - USB OTG High Speed
    CONFIG_SAM34_TRNG        - True Random Number Generator
    CONFIG_SAM34_EMAC        - Ethernet MAC
    CONFIG_SAM34_CAN0        - CAN Controller 0
    CONFIG_SAM34_CAN1        - CAN Controller 1

  Some subsystems can be configured to operate in different ways. The drivers
  need to know how to configure the subsystem.

    CONFIG_GPIOA_IRQ
    CONFIG_GPIOB_IRQ
    CONFIG_GPIOC_IRQ
    CONFIG_GPIOD_IRQ
    CONFIG_GPIOE_IRQ
    CONFIG_GPIOF_IRQ
    CONFIG_USART0_ISUART
    CONFIG_USART1_ISUART
    CONFIG_USART2_ISUART
    CONFIG_USART3_ISUART

  ST91SAM4S specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=0,1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

  Each SAM4S Xplained configuration is maintained in a sub-directory and
  can be selected as follow:

    cd tools
    ./configure.sh arduino-due/<subdir>
    cd -
    . ./setenv.sh

  Before sourcing the setenv.sh file above, you should examine it and perform
  edits as necessary so that BUILDROOT_BIN is the correct path to the directory
  than holds your toolchain binaries.

  And then build NuttX by simply typing the following.  At the conclusion of
  the make, the nuttx binary will reside in an ELF file called, simply, nuttx.

    make

  The <subdir> that is provided above as an argument to the tools/configure.sh
  must be is one of the following.

  NOTES:

  1. These configurations use the mconf-based configuration tool.  To
    change any of these configurations using that tool, you should:

    a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
       and misc/tools/

    b. Execute 'make menuconfig' in nuttx/ in order to start the
       reconfiguration process.

  2. Unless stated otherwise, all configurations generate console
     output on UART1 which is available on J1 or J4 (see the
     section "Serial Consoles" above).  USART1 or the virtual COM
     port on UART0 are options. The virtual COM port could
     be used, for example, by reconfiguring to use UART0 like:

       System Type -> AT91SAM3/4 Peripheral Support
         CONFIG_SAM_UART0=y
         CONFIG_SAM_UART1=n

       Device Drivers -> Serial Driver Support -> Serial Console
         CONFIG_UART0_SERIAL_CONSOLE=y

       Device Drivers -> Serial Driver Support -> UART0 Configuration
         CONFIG_UART0_2STOP=0
         CONFIG_UART0_BAUD=115200
         CONFIG_UART0_BITS=8
         CONFIG_UART0_PARITY=0
         CONFIG_UART0_RXBUFSIZE=256
         CONFIG_UART0_TXBUFSIZE=256

  3. Unless otherwise stated, the configurations are setup for
     Linux (or any other POSIX environment like Cygwin under Windows):

     Build Setup:
       CONFIG_HOST_LINUX=y   : Linux or other POSIX environment

  4. These configurations use the older, OABI, buildroot toolchain.  But
     that is easily reconfigured:

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

     If you want to use the Atmel GCC toolchain, here are the steps to
     do so:

     Build Setup:
       CONFIG_HOST_WINDOWS=y   : Windows
       CONFIG_HOST_CYGWIN=y    : Using Cygwin or other POSIX environment

     System Type -> Toolchain:
       CONFIG_ARMV7M_TOOLCHAIN_GNU_EABIW=y : General GCC EABI toolchain under windows

     This re-configuration should be done before making NuttX or else the
     subsequent 'make' will fail.  If you have already attempted building
     NuttX then you will have to 1) 'make distclean' to remove the old
     configuration, 2) 'cd tools; ./configure.sh sam3u-ek/ksnh' to start
     with a fresh configuration, and 3) perform the configuration changes
     above.

     Also, make sure that your PATH variable has the new path to your
     Atmel tools.  Try 'which arm-none-eabi-gcc' to make sure that you
     are selecting the right tool.  setenv.sh is available for you to
     use to set or PATH variable.  The path in the that file may not,
     however, be correct for your installation.

     See also the "NOTE about Windows native toolchains" in the section call
     "GNU Toolchain Options" above.

Configuration sub-directories
-----------------------------

  ostest:
    This configuration directory performs a simple OS test using
    examples/ostest.  See NOTES above.

  nsh:
    This configuration directory will built the NuttShell.  See NOTES above.

    NOTES:
    1. The configuration configuration can be modified to include support
       for the on-board SRAM (1MB).

       System Type -> External Memory Configuration       
         CONFIG_ARCH_EXTSRAM0=y              : Select SRAM on CS0
         CONFIG_ARCH_EXTSRAM0SIZE=1048576    : Size=1MB

       Now what are you going to do with the SRAM.  There are two choices:

       a)  To enable the NuttX RAM test that may be used to verify the
           external SRAM:

           System Type -> External Memory Configuration       
             CONFIG_ARCH_EXTSRAM0HEAP=n      : Don't add to heap

           Application Configuration -> System NSH Add-Ons
             CONFIG_SYSTEM_RAMTEST=y         : Enable the RAM test built-in

         In this configuration, the SDRAM is not added to heap and so is
         not excessible to the applications.  So the RAM test can be
         freely executed against the SRAM memory beginning at address
         0x6000:0000 (CS0).

         nsh> ramtest -h
         Usage: <noname> [-w|h|b] <hex-address> <decimal-size>

         Where:
           <hex-address> starting address of the test.
           <decimal-size> number of memory locations (in bytes).
           -w Sets the width of a memory location to 32-bits.
           -h Sets the width of a memory location to 16-bits (default).
           -b Sets the width of a memory location to 8-bits.

         To test the entire external SRAM:

         nsh> ramtest 60000000 1048576
         RAMTest: Marching ones: 60000000 1048576
         RAMTest: Marching zeroes: 60000000 1048576
         RAMTest: Pattern test: 60000000 1048576 55555555 aaaaaaaa
         RAMTest: Pattern test: 60000000 1048576 66666666 99999999
         RAMTest: Pattern test: 60000000 1048576 33333333 cccccccc
         RAMTest: Address-in-address test: 60000000 1048576

        b) To add this RAM to the NuttX heap, you would need to change the
           configuration as follows:

           System Type -> External Memory Configuration       
             CONFIG_ARCH_EXTSRAM0HEAP=y     : Add external RAM to heap

           Memory Management
             -CONFIG_MM_REGIONS=1           : Only the internal SRAM
             +CONFIG_MM_REGIONS=2           : Also include external SRAM
