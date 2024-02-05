================
``i2c`` I2C Tool
================

The I2C tool provides a way to debug I2C related problems. This README file will
provide usage information for the I2C tools.

Contents
--------

- System Requirements
  - I2C Driver
  - Configuration Options
- Help
- Common Line Form
- Common Command Options
  - _Sticky_ Options
  - Environment variables
  - Common Option Summary
- Command summary
  - ``bus``
  - ``dev``
  - ``get``
  - ``set``
  - ``verf``
- I2C Build Configuration
  - NuttX Configuration Requirements
  - I2C Tool Configuration Options

System Requirements
-------------------

The I2C tool is designed to be implemented as a NuttShell (NSH) add-on. Read the
``apps/nshlib/README.md`` file for information about add-ons.

Configuration Options
~~~~~~~~~~~~~~~~~~~~~

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the tools as an NSH built-in command.
- ``CONFIG_I2CTOOL_MINBUS`` – Smallest bus index supported by the hardware
  (default ``0``).
- ``CONFIG_I2CTOOL_MAXBUS`` – Largest bus index supported by the hardware
  (default ``3``).
- ``CONFIG_I2CTOOL_MINADDR`` – Minimum device address (default: ``0x03``).
- ``CONFIG_I2CTOOL_MAXADDR`` – Largest device address (default: ``0x77``).
- ``CONFIG_I2CTOOL_MAXREGADDR`` – Largest register address (default: ``0xff``).
- ``CONFIG_I2CTOOL_DEFFREQ`` – Default frequency (default: ``4000000``).

Help
----

First of all, the I2C tools supports a pretty extensive help output. That help
output can be view by entering either::

  nsh> i2c help

or::

  nsh> i2c ?

Here is an example of the help output. I shows the general form of the command
line, the various I2C commands supported with their unique command line options,
and a more detailed summary of the command I2C command options::

  nsh> i2c help

  Usage: i2c <cmd> [arguments]
  Where <cmd> is one of:

    Show help     : ?
    List buses    : bus
    List devices  : dev [OPTIONS] <first> <last>
    Read register : get [OPTIONS] [<repetitions>]
    Show help     : help
    Write register: set [OPTIONS] <value> [<repetitions>]
    Verify access : verf [OPTIONS] <value> [<repetitions>]

    Where common _sticky_ OPTIONS include:
      [-a addr] is the I2C device address (hex).  Default: 03 Current: 03
      [-b bus] is the I2C bus number (decimal).  Default: 1 Current: 1
      [-r regaddr] is the I2C device register address (hex).  Default: 00 Current: 00
      [-w width] is the data width (8 or 16 decimal).  Default: 8 Current: 8
      [-s|n], send/don't send start between command and data.  Default: -n Current: -n
      [-i|j], Auto increment|don't increment regaddr on repetitions.  Default: NO Current: NO
      [-f freq] I2C frequency.  Default: 100000 Current: 100000

**Notes**:

- An environment variable like ``$PATH`` may be used for any argument.
- Arguments are _sticky_. For example, once the I2C address is specified, that
  address will be re-used until it is changed.

**Warning**:

- The I2C dev command may have bad side effects on your I2C devices. Use only at
  your own risk.

Command Line Form
-----------------

The I2C is started from NSH by invoking the ``i2c`` command from the NSH command
line. The general form of the ``i2c`` command is::

  i2c <cmd> [arguments]

Where ``<cmd>`` is a sub-command and identifies one I2C operations supported by
the tool. ``[arguments]`` represents the list of arguments needed to perform the
I2C operation. Those arguments vary from command to command as described below.
However, there is also a core set of common ``OPTIONS`` supported by all commands.
So perhaps a better representation of the general I2C command would be::

  i2c <cmd> [OPTIONS] [arguments]

Where ``[OPTIONS]`` represents the common options and and arguments represent the
operation-specific arguments.

Common Command Options
----------------------

"Sticky" Options
~~~~~~~~~~~~~~~~

In order to interact with I2C devices, there are a number of I2C parameters that
must be set correctly. One way to do this would be to provide to set the value
of each separate command for each I2C parameter. The I2C tool takes a different
approach, instead: The I2C configuration can be specified as a (potentially
long) sequence of command line arguments.

These arguments, however, are _sticky_. They are sticky in the sense that once
you set the I2C parameter, that value will remain until it is reset with a new
value (or until you reset the board).

Environment Variables
~~~~~~~~~~~~~~~~~~~~~

**Note** also that if environment variables are not disabled (by
``CONFIG_DISABLE_ENVIRON=y``), then these options may also be environment
variables. Environment variables must be preceded with the special character
``$``. For example, ``PWD`` is the variable that holds the current working directory
and so ``$PWD`` could be used as a command line argument. The use of environment
variables on the I2C tools command is really only useful if you wish to write
NSH scripts to execute a longer, more complex series of I2C commands.

Common Option Summary
~~~~~~~~~~~~~~~~~~~~~

- ``[-a addr]`` is the I2C device address (hex). Default: ``03`` Current: ``03``

  The ``[-a addr]`` sets the I2C device address. The valid range is ``0x03`` through
  ``0x77`` (this valid range is controlled by the configuration settings
  ``CONFIG_I2CTOOL_MINADDR`` and ``CONFIG_I2CTOOL_MAXADDR``). If you are working
  with the same device, the address needs to be set only once.

  All I2C address are 7-bit, hexadecimal values.

  **Note 1**: Notice in the ``help`` output above it shows both default value of the
  I2C address (``03`` hex) and the current address value (also ``03`` hex).

  **Note 2**: Sometimes I2C addresses are represented as 8-bit values (with bit zero
  indicating a read or write operation). The I2C tool uses a 7-bit
  representation of the address with bit 7 unused and no read/write indication
  in bit 0. Essentially, the 7-bit address is like the 8-bit address shifted
  right by 1.

  **Note 3**: Most I2C bus controllers will also support 10-bit addressing. That
  capability has not been integrated into the I2C tool as of this writing.

- ``[-b bus]`` is the I2C bus number (decimal). Default: ``1`` Current: ``1``

  Most devices support multiple I2C devices and also have unique bus numbering.
  This option identifies which bus you are working with now. The valid range of
  bus numbers is controlled by the configuration settings
  ``CONFIG_I2CTOOL_MINBUS`` and ``CONFIG_I2CTOOL_MAXBUS``.

  The bus numbers are small, decimal numbers.

- ``[-r regaddr]`` is the I2C device register address (hex). Default: ``00``
  Current: ``00``

  The I2C set and get commands will access registers on the I2C device. This
  option selects the device register address (sometimes called the sub-address).
  This is an 8-bit hexadecimal value. The maximum value is determined by the
  configuration setting ``CONFIG_I2CTOOL_MAXREGADDR``.

- ``[-w width]`` is the data width (8 or 16 decimal). Default: ``8`` Current: ``8``

  Device register data may be 8-bit or 16-bit. This options selects one of those
  two data widths.

- ``[-s|n]``, send/don't send start between command and data. Default: ``-n``
  Current: ``-n``

  This determines whether or not there should be a new I2C START between sending
  of the register address and sending/receiving of the register data.

- ``[-i|j]``, Auto increment|don't increment ``regaddr`` on repetitions. Default:
  ``NO`` Current: ``NO``

  On commands that take a optional number of repetitions, the option can be used
  to temporarily increment the ``regaddr`` value by one on each repetition.

- ``[-f freq]`` I2C frequency. Default: ``400000`` Current: ``400000``

  The ``[-f freq]`` sets the frequency of the I2C device.

Command Summary
---------------

We have already seen the I2C help (or ``?``) commands above. This section will
discuss the remaining commands.

List buses: ``bus [OPTIONS]``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This command will simply list all of the configured I2C buses and indicate which
are supported by the driver and which are not::

  BUS   EXISTS?
  Bus 1: YES
  Bus 2: NO

The valid range of bus numbers is controlled by the configuration settings
``CONFIG_I2CTOOL_MINBUS`` and ``CONFIG_I2CTOOL_MAXBUS``.

List devices: ``dev [OPTIONS] <first> <last>``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``dev`` command will attempt to identify all of the I2C devices on the
selected bus. The ``<first>`` and ``<last>`` arguments are 7-bit, hexadecimal I2C
addresses. This command will examine a range of addresses beginning with
``<first>`` and continuing through ``<last>``. It will request the value of register
address zero from each device.

The register address of zero is always used by default. The previous _sticky_
register address is ignored. Some devices may not respond to ergister address
zero, however. To work around this, you can provide a new _sticky_ register
address on the command as an option to the 'dev' command. Then that new _sticky_
register address will be used instead of the address zero.

If the device at an I2C address responds to the read request, then the ``dev``
command will display the I2C address of the device. If the device does not
respond, this command will display ``--``. The resulting display looks like::

  nsh> i2c dev 03 77

       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
  00:         -- -- -- -- -- -- -- -- -- -- -- -- --
  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  40: -- -- -- -- -- -- -- -- -- 49 -- -- -- -- -- --
  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  70: -- -- -- -- -- -- -- --

Warnings:

- The I2C dev command may have bad side effects on certain I2C devices. For
  example, if could cause data loss in an EEPROM device.

- The I2C dev command also depends upon the underlying behavior of the I2C
  driver. How does the driver respond to addressing failures?

Read register: ``get [OPTIONS]``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This command will read the value of the I2C register using the selected I2C
parameters in the common options. No other arguments are required.

This command with write the 8-bit address value then read an 8- or 16-bit data
value from the device. Optionally, it may re-start the transfer before obtaining
the data.

An optional ``<repetitions>`` argument can be supplied to repeat the read
operation an arbitrary number of times (up to 2 billion). If auto-increment is
select (``-i``), then the register address will be temporarily incremented on each
repetitions. The increment is temporary in the since that it will not alter the
_sticky_ value of the register address.

On success, the output will look like the following (the data value read will be
shown as a 4-character hexadecimal number if the 16-bit data width option is
selected)::

  READ Bus: 1 Addr: 49 Subaddr: 04 Value: 96

All values (except the bus numbers) are hexadecimal.

Write register: ``set [OPTIONS] <value>``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This command will write a value to an I2C register using the selected I2C
parameters in the common options. The value to write must be provided as the
final, hexadecimal value. This value may be an 8-bit value (in the range
``00``-``ff``) or a 16-bit value (in the range ``0000``-``ffff``), depending upon the
selected data width.

This command will write the 8-bit address value then write the 8- or 16-bit data
value to the device. Optionally, it may re-start the transfer before writing the
data.

An optional ``<repetitions>`` argument can be supplied to repeat the write
operation an arbitrary number of times (up to 2 billion). If auto-increment is
select (``-i``), then the register address will be temporarily incremented on each
repetitions. The increment is temporary in the since that it will not alter the
_sticky_ value of the register address.

On success, the output will look like the following (the data value written will
be shown as a 4-character hexadecimal number if the 16-bit data width option is
selected)::

  WROTE Bus: 1 Addr: 49 Subaddr: 04 Value: 96

All values (except the bus numbers) are hexadecimal.

Verify access: ``verf [OPTIONS] <value> [<repetitions>]``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This command combines writing and reading from an I2C device register. It will
write a value to an will write a value to an I2C register using the selected I2C
parameters in the common options just as described for tie ``set`` command. Then
this command will read the value back just as described with the ``get`` command.
Finally, this command will compare the value read and against the value written
and emit an error message if they do not match.

If no value is provided, then this command will use the register address itself
as the data, providing for a address-in-address test.

An optional ``<repetitions>`` argument can be supplied to repeat the verify
operation an arbitrary number of times (up to 2 billion). If auto-increment is
select (``-i``), then the register address will be temporarily incremented on each
repetitions. The increment is temporary in the since that it will not alter the
``sticky`` value of the register address.

On success, the output will look like the following (the data value written will
be shown as a 4-character hexadecimal number if the 16-bit data width option is
selected)::

  VERIFY Bus: 1 Addr: 49 Subaddr: 04 Wrote: 96 Read: 92 FAILURE

All values (except the bus numbers) are hexadecimal.

I2C Build Configuration
-----------------------

NuttX Configuration Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The I2C tools requires the following in your NuttX configuration:

1. Application configuration.

   Using ``make menuconfig``, select the i2c tool. The following definition should
   appear in your ``.config`` file::

     CONFIG_SYSTEM_I2C=y

2. Device-specific I2C driver support must be enabled::

     CONFIG_I2C_DRIVER=y

   The I2C tool will then use the I2C character driver to access the I2C bus.
   These devices will reside at ``/dev/i2cN`` where ``N`` is the I2C bus number.

   **Note**: The I2C driver ``ioctl`` interface is defined in
   ``include/nuttx/i2c/i2c_master.h``.

I2C Tool Configuration Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The default behavior of the I2C tool can be modified by the setting the options
in the NuttX configuration. This configuration is the ``defconfig`` file in your
configuration directory that is copied to the NuttX top-level directory as
``.config`` when NuttX is configured.

- ``CONFIG_NSH_BUILTIN_APPS`` – Build the tools as an NSH built-in command.
- ``CONFIG_I2CTOOL_MINBUS`` – Smallest bus index supported by the hardware
  (default ``0``).
- ``CONFIG_I2CTOOL_MAXBUS`` – Largest bus index supported by the hardware
  (default ``3``).
- ``CONFIG_I2CTOOL_MINADDR`` – Minimum device address (default: ``0x03``).
- ``CONFIG_I2CTOOL_MAXADDR`` – Largest device address (default: ``0x77``).
- ``CONFIG_I2CTOOL_MAXREGADDR`` – Largest register address (default: ``0xff``).
- ``CONFIG_I2CTOOL_DEFFREQ`` – Default frequency (default: ``4000000``).
