README
======

  Overview
  --------
  Initial port to ESP32C3-Devkit-RUST-1 board:
  https://github.com/esp-rs/esp-rust-board

  Testing
  -------

    $ ./tools/configure.sh esp32c3-devkit-rust-1:nsh
    $ make
    $ make flash ESPTOOL_PORT=/dev/ttyACM0

  Alternatively you can compile and flash faster this way:

    $ make -j flash ESPTOOL_PORT=/dev/ttyACM0

  After flashing NuttX in this board use minicom or other serial console
  configured to:

    Device: /dev/ttyACM0
    Baudrate: 9600 8n1

  If everything is fine you should be able to get the serial console:

    nsh> uname -a
    NuttX 10.4.0 55df6e951e-dirty Aug 22 2022 17:12:29 risc-v esp32c3-devkit-rust-1

    nsh> mount
      /proc type procfs

    nsh> free
                       total       used       free    largest  nused  nfree
            Umem:     377872       5792     372080     372080     30      1
    nsh>
