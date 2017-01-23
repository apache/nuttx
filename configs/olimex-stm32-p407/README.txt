README
======

The NuttX configuration for the Olimex STM32-P407 is derives more or less
directly from the Olimex STM32-P207 board support.  The P207 and P407 seem
to share the same board design.  Other code comes from the STM3240G board
support (which has the same crystal and clocking) and from the STM32 F4
Discovery (which has the same STM32 part)

Note that CONFIG_STM32_DISABLE_IDLE_SLEEP_DURING_DEBUG is enabled so
that the JTAG connection is not disconnected by the idle loop.

The following peripherals are available in this configuration.

 - LEDs:       show the sytem status

 - Buttons:    TAMPER-button, WKUP-button, J1-Joystick (consists of RIGHT-,
               UP-, LEFT-, DOWN-, and CENTER-button). Built in app
               'buttons' works.

 - ADC:        ADC1 samples the red trim potentiometer AN_TR
               Built in app 'adc' works.

 - USB-FS-OTG: enabled but not really tested, since there is only a
               USB-A-connector (host) connected to the full speed µC inputs.
               The other connector (device) is connected to the high speed µC
               inputs, but it seems that NuttX has currently no driver
               for it.

 - CAN:        Built in app 'can' works, but apart from that not really tested.

 - Ethernet:   Ping to other station on the network works.
Configurations
==============

Each Olimex STM32-P407 configuration is maintained in a sub-directory and can be
selected as follow:

    cd tools
    ./configure.sh olimex-stm32-p407/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:

    This is the NuttShell (NSH) using the NSH startup logic at
    apps/examples/nsh.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configurations using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          see additional README.txt files in the NuttX tools repository.

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Serial Output

       This configuraiont produces all of its test output on the serial
       console.  This configuration has USART3 enabled as a serial console.
       This is the connector labeled RS232_2.  This can easily be changed
       by reconfiguring with 'make menuconfig'.

    3. Toolchain

       By default, the host platform is set to be Linux using the NuttX
       buildroot toolchain. The host and/or toolchain selection can easily
       be changed with 'make menuconfig'.

    4. Kernel Modules

       I used this configuration for testing NuttX kernel modules with the
       following configuration additions to the configuration file:

          CONFIG_BOARDCTL_OS_SYMTAB=y
          CONFIG_EXAMPLES_MODULE=y
          CONFIG_EXAMPLES_MODULE_DEVMINOR=0
          CONFIG_EXAMPLES_MODULE_DEVPATH="/dev/ram0"
          CONFIG_FS_ROMFS=y
          CONFIG_LIBC_ARCH_ELF=y
          CONFIG_LIBC_DLLFCN=y
          CONFIG_MODULE=y
          CONFIG_MODULE_ALIGN_LOG2=2
          CONFIG_MODULE_BUFFERINCR=32
          CONFIG_MODULE_BUFFERSIZE=128

STATUS:

2016-12-21: This board configuration was ported from the Olimex STM32 P207
  port.  Note that none of the above features have been verified.  USB, CAN,
  ADC, and Ethernet are disabled in the base NSH configuration until they
  can be verified.  These features should be functional but may required
  some tweaks due to the different clock configurations.  The Olimex STM32
  P207 nsh/defconfig would be a good starting place for restoring these
  feature configurations.

  CCM memory is not included in the heap (CONFIG_STM32_CCMEXCLUDE=y) because
  it does no suport DMA, leaving only 128KiB for program usage.
