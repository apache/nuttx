README
======

gapuino is an evaluation board for GAP8, a 1+8-core DSP-like RISC-V
MCU. GAP8 features a RI5CY core called Fabric Controller(FC), and a
cluster of 8 RI5CY cores that runs at a bit slower speed. GAP8 is an
implementation of the opensource PULP platform, a Parallel-Ultra-Low-
Power design.

The port is currently very minimal, though additional support may be
added in the future to address more of the board peripherals.

  Supported:
  - USB UART (console port)
  - uDMA on SOC domain
  - FLL clock scaling

  Not supported:
  - SPI, I2C, I2S, CPI, LVDS, Hyper-bus on the uDMA subsystem
  - the sensor board
  - the 8-core cluster
  - the Hardware convolution engine

See also:
gapuino board and the sensor board:
https://gwt-website-files.s3.amazonaws.com/gapuino_um.pdf
https://gwt-website-files.s3.amazonaws.com/gapuino_multisensor_um.pdf
GAP8 datasheet:
https://gwt-website-files.s3.amazonaws.com/gap8_datasheet.pdf

Contents
========

  - Environment Setup
  - Configurations
  - Execute

Environment Setup
=================
  First, setup the gap_sdk from GreenwavesTechnologies' github repo.
  Follow the README to setup the toolchain and environment.
  https://github.com/GreenWaves-Technologies/gap_sdk/

Configurations
==============
  Each gapuino configuration is maintained in a sub-directory and can
  be selected as follow:

    tools/configure.sh gapuino:<subdir>

  Where <subdir> is one of the following:

    nsh
    ---
    This is an NSH example that uses the UART connected to FT2232 as
    the console. Default UART settings are 115200, 8N1.

Execute
=======
  You may download the ELF to the board by `plpbridge` in gap_sdk.
  Remember to first `cd` to the gap_sdk/ and `source sourceme.sh`, so
  that you have the $GAP_SDK_HOME environment variable.

  Use the following command to download and run the ELF through JTAG:

    $GAP_SDK_HOME/install/workstation/bin/plpbridge \
    --cable=ftdi@digilent --boot-mode=jtag --chip=gap \
    --binary=nuttx \
    load ioloop reqloop start wait

  As for debugging, the following command download the ELF and opens
  a gdbserver on port 1234:

     $GAP_SDK_HOME/install/workstation/bin/plpbridge \
    --cable=ftdi@digilent --boot-mode=jtag --chip=gap \
    --binary=nuttx \
    load ioloop gdb wait

  And then launch the gdb on another terminal:

    riscv32-unknown-elf-gdb nuttx
    ...
    (gdb) target remote :1234
    Remote debugging using :1234
    IRQ_U_Vector_Base () at chip/startup_gap8.S:293
    293             j reset_handler       /* 32 */
    (gdb)

  And then enjoy yourself debugging with the CLI gdb :-)
