README
======
The Olimex STM32-E407 configuration is based on the configuration olimex-stm32-h407 and stm32f4discovery.

Configurations
==============

  Instantiating Configurations
  ----------------------------
  Each Olimex-STM32-E407 configuration is maintained in a sub-directory and
  can be selected as follow:

    tools/configure.sh olimex-stm32-e407/<subdir>

  Where <subdir> is one of the following:

  Compile Firmware
  ----------------
  Once you've set the proper configuration, you just need to execute the next
  command:

     make

  If everything goes find, it should return the next two files:

    nuttx.hex
    nuttx.bin

  You can return more kind of files by setting on menuconfig.


  Flashing the Board
  -----------------
  You can flash this board in different ways, but the easiest way is using
  ARM-USB-TINY-H JTAG flasher device.
  Connect this device to the JTAG connector and type the next command:

    openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"

  Configuration Directories
  -------------------------
  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a console on UART2. Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

  usbnsh:
  ------
    Configures the NuttShell (nsh) located at apps/examples/nsh. This
    configuration enables a console on USB_OTG1. Support for
    builtin applications is enabled, but in the base configuration no
    builtin applications are selected.

  netnsh:
  ------
    Configures the NuttShell (nsh) located at examples/nsh.  This
    configuration is focused on network testing.

  BMP180:
  ------
   This a configuration example for the BMP180 barometer sensor. This
   sensor works with I2C, you need to do the next connections:

   BMP180 VIN -> Board 3.3V
   BMP180 GND -> Board GND
   BMP180 SCL -> Board PB6 (Arduino header D1)
   BMP180 SDA -> Board PB7 (Arduino header D0)

   This example is configured to work with the USBNSH instead of UART NSH, so
   the console will be shown over the USB_OTG1 connector.

   On the console, type "ls /dev " and if the registration process goes fine,
   you should see a device called "press0". Now execute the app
   BMP180 to see the ambient pressure value.
