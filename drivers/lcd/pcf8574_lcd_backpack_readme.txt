pcf8574 lcd backpack - readme.txt
20160524a, ziggurat29

Abstract
========

This describes the use of the pcf8574_lcd_backpack.h, .c driver module for NuttX.

Contents
========

  o Summary for Those Who Don't Like to Read
  o Introduction
  o Usage
    - Specifying the I2C Address
    - Specifying the LCD Display Format
    - Specifying Unknown/New Backpacks
  o Special Features
    - Codec
    - Ioctl
  o Troubleshooting

Summary for Those Who Don't Like to Read
========================================

To use, in your board_app_initialize(),

1)  instantiate an I2C bus:

    FAR struct i2c_master_s* i2c = stm32l4_i2cbus_initialize(1);

2)  set the configuration for the particular make of board, and LCD format:

    struct pcf8574_lcd_backpack_config_s cfg = LCD_I2C_BACKPACK_CFG_MJKDZ;
    cfg.rows = 2;
    cfg.cols = 16;

3)  instantiate the device on the I2C bus previously created:

    ret = pcf8574_lcd_backpack_register("/dev/slcd0", i2c, &cfg);

Introduction
============

The character LCD modules based on the HD44780 (and compatible ST7706U, KS0066U,
SED1278, etc.) drivers have been around for many decades and are quite popular.
One challenge is that they require a large number of GPIO (11 in 8-bit mode, 7
in 4-bit mode, and an additional line if you control the backlight).

To address this, several folks have created daughter boards for the LCD module
which present a two-wire I2C interface.  Generally, folks call these interface
boards an 'lcd backpack'.  A large class of them (and in particular, the very
inexpensive ones found on ebay, q.v. google "ebay i2c lcd backpack"; they're
usually about $USD 1), use the same design:  a PCF8574 I2C IO expander.
Variations occur in mapping GPIO line to LCD pins, but otherwise the
expectation is that you control the LCD at a low-level tweaking the lines
("byte-banging"?)

My original motivation for producing this was to simply serve as a test device
for some I2C driver work I was doing, but it occurred to me that it may be
useful to others, given the popularity of the 'lcd backpack', so I cleaned up
the code and made it general to support all the variations on the market, and
also to adopt the NuttX notion of a 'segment lcd codec', which is used to
transport escape sequences (for doing things like clearing the display, turning
on/off the cursor, etc), and also the standard ioctls.

I believe it should support all "lcd backpack"s on the market (because you can
specify the particular wiring), and all HD44780-based LCD modules in 1-line,
2-line, and 4-line configurations (except 4x40 -- this is not supported by
the hardware).

This module should be cpu-architecture-neutral, and work with any standard I2C
bus object.  At the time of this writing it has been tested only with the
STM32L4 chip and with the 'MJKDZ' backpack board with a 16x2 lcd module.

Usage
=====

The driver is contained in the files pcf8574_lcd_backpack.h and
pcf8574_lcd_backpack.c; you can include these in your build in whatever manner
you choose (e.g. copy them into your board's src directory, and reference them
in the Makefile).

As with other I2C devices, you first instantiate the I2C bus, and then
instantiate the driver on that bus.  When instantiating the driver, you also
provide a configuration 'descriptor' that specified board wiring and LCD
format parameters.  You can explicitly specify any wiring configuration, and
some known popular boards are already #defined for your convenience.

E.g.:

    #include <nuttx/i2c/i2c_master.h>
    #include "pcf8574_lcd_backpack.h"

    #define MJKDZ_I2C_PORTNO 1
    #define MJKDZ_DEVICE_NAME "/dev/lcd0"

    FAR struct i2c_master_s* g_i2cMJKDZ = NULL;

    ....

    g_i2cMJKDZ = stm32l4_i2cbus_initialize(MJKDZ_I2C_PORTNO);

    ....

    struct pcf8574_lcd_backpack_config_s cfg = LCD_I2C_BACKPACK_CFG_MJKDZ;
    cfg.rows = 2;
    cfg.cols = 16;

    ret = pcf8574_lcd_backpack_register(MJKDZ_DEVICE_NAME, g_i2cMJKDZ, &cfg);

If all the above executes successfully, you should wind up with a character
device node "/dev/lcd0".  Applications can open that node and write() to it,
and the shell can emit data to it (e.g. 'echo Hi, there! > /dev/lcd0').

That is the basic configuration.  Some additional configuration points are
worth noting.

Specifying the I2C Address
--------------------------

The 'struct pcf8574_lcd_backpack_config_s' shown above is initialized using
the convenience macro LCD_I2C_BACKPACK_CFG_MJKDZ.  Those convenience macros
use the default I2C address for the board, however many of the boards allow
altering the address (by jumpers, or removing pullups).  You need to specify
the correct address for your board's physical configuration.  You can do that
via

  cfg.addr = 0x23;

Specifying the LCD Display Format
---------------------------------

The LCD modules cannot 'self-describe' their physical format, so it must be
explicitly provided to the driver.  The correct format is important for
computing screen coordinate addresses and for scrolling and line wrap.

In the example above, the screen format is specifying by setting the
fields in the configuration descriptor:

  cfg.rows = 2;
  cfg.cols = 16;

The lcd backpack can accommodate all known 1-line and 2-line displays, and
4-line displays up to 4 x 32.  Explicitly, the 4 x 40 /cannot/ be supported
because it has an important hardware difference (it is actually two 4x20
controllers, and the LCD backpack does not have the wiring for the
second controller's 'E' line).  This is a hardware limitation of the
lcd backpack, rather than the driver.

Specifying Unknown/New Backpacks
--------------------------------

The descriptor initializer macros in the form  LCD_I2C_BACKPACK_CFG_xxx
located near the top of pcf8574_lcd_backpack.h are provided for convenience.
However, their use is not required, and it can be useful to initialize the
descriptor with explicit values, say, for custom or unknown boards.

The format of this descriptor is conscientiously chosen to be semantically
similar to an equivalent initialization mechanism popular in the Arduino
community used in their LCD support libraries.  It specifies:

  * I2C address
  * pin mapping for data lines
  * pin mapping for control lines
  * pin mapping for backlight control line
  * polarity sense of backlight control line

and we add to that

  * (row, column) size of display

(the Arduino libraries specify display size at a different point in code)
You should be able to readily port a functional Arduino project by cutting-
and-pasting the sequence of numbers that are the pin defs for the lcd
backpack you are using.

Special Features
================

Codec
-----

The driver supports the NuttX 'segment lcd codec', which facilitates the
encoding of control functions into the write() stream.  These can be used
to clear the display, move the cursor, etc.  For details, q.v.

    nuttx/lcd/slcd_codec.h

Ioctl
-----

The driver supports the NuttX ioctl definitions for segment lcd.  Q.v.

    nuttx/lcd/slcd_ioctl.h

Additionally, the ioctl SLCDIOC_CREATECHAR is provided to allow the
creation of custom characters.

The HD44780 devices generally support the creation of 8 custom
characters, which map to code points 0-7.  The characters are 5x8
pixels (with the expectation that the last row is left blank, to
accommodate the underscore cursor, though this is not strictly a
requirement).

The SLCDIOC_CREATECHAR ioctl takes a parameter, which is a struct
consisting of the character index being programmed (0-7) and the
8-byte bitmap of the character image.  The bitmap is constructed
with each byte representing a row, from top row to bottom row.
Each row is imaged left to right, MSB to LSB, right-justified (i.e.,
bit 4 is leftmost, bit 0 is rightmost, and bits 7-5 are unused).

You may reference these characters simply by including them in
the data you write() to the device, e.g.

    write(fd, "\x01,\x02Hi, there!\n", 13);

Example of programming a character image:

  static const struct slcd_createchar_s custom_char =
      { 4, { 0x04, 0x0e, 0x15, 0x04, 0x04, 0x04, 0x04, 0x00 } };  /* up arrow */

  ret = ioctl(fd, SLCDIOC_CREATECHAR, (unsigned long)custom_char);

Now character '\x04' will display as an 'up arrow'.

Note, you might consider avoiding the use of code point 0x00 unless
you absolutely need it, because the embedded nul character can cause
problems.  The driver, and write() apis are binary, and unaffected,
but things like printf() and puts() assume C-style strings, and are
affected.

Troubleshooting
===============

* Check your I2C address.  turn on debugging output so you can see
  bus timeouts that suggest a non-responsive slave.
* Check your board wiring and configuration specification.  Buzz
  out the lines if you have to.
* Remember to set the (ros,cols) geometry in pcf8574_lcd_backpack_config_s
  before registration of the driver, since this cannot be determined
  programmatically.
* If the driver registration step seems to 'hang' it could be the I2C
  driver performing retries due to no response from the LCD backpack.  Check
  the address.  Turning on debug output for I2C can help make this visible.
* Don't forget to check the 'contrast' potentiometer.  The voltage at the
  central wiper should be approximately 0.3 V - 2.4 V, but the actual value
  is is dependent on the physics of the attached LCD module.  The useful
  range of voltages at this pin for any given LCD is quite narrow, and
  outside that range there will be nothing visible on the display, so most
  of the turn range of the pot is useless.  It's less 'contrast' and
  more 'LCD segment drive bias'.
