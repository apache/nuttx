=================
Microchip AT90USB
=================

AVR AT90USB64x and AT90USB6128x
-------------------------------

**Micropendous 3 AT90USB64x** and **AT90USB6128x**. This port of NuttX
to the Opendous Micropendous 3 board. The Micropendous3 is may be
populated with an AT90USB646, 647, 1286, or 1287. I have only the
AT90USB647 version for testing. This version have very limited memory
resources: 64K of FLASH and 4K of SRAM.

**PJRC Teensy++ 2.0 AT90USB1286**. This is a port of NuttX to the PJRC
Teensy++ 2.0 board. This board was developed by
`PJRC <http://pjrc.com/teensy/>`__. The Teensy++ 2.0 is based on an
Microchip AT90USB1286 MCU.

**AVR-Specific Issues**. The basic AVR port is solid. The biggest issue
for using AVR is its tiny SRAM memory and its Harvard architecture.
Because of the Harvard architecture, constant data that resides to flash
is inaccessible using "normal" memory reads and writes (only SRAM data
can be accessed "normally"). Special AVR instructions are available for
accessing data in FLASH, but these have not been integrated into the
normal, general purpose OS.

Most NuttX test applications are console-oriented with lots of strings
used for ``printf()`` and debug output. These strings are all stored in
SRAM now due to these data accessing issues and even the smallest
console-oriented applications can quickly fill a 4-8K memory. So, in
order for the AVR port to be useful, one of two things would need to be
done:

#. Don't use console applications that required lots of strings. The
   basic AVR port is solid and your typical deeply embedded application
   should work fine. Or,
#. Create a special version of printf that knows how to access strings
   that reside in FLASH (or EEPROM).

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain, or 4) Native Windows. All testing, however, has been
performed using the NuttX DIY toolchain for Linux or Cygwin is provided
by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package. As a result, that toolchain is recommended.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
