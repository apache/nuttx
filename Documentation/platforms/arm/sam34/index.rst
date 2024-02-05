===============
Microchip SAM34
===============

Microchip SAM3U
---------------

This port uses the `Microchip <http://www.atmel.com/>`__
SAM3U-EK development board that features the SAM3U4E MCU. This port uses
a GNU arm-nuttx-elf or arm-nuttx-eabi toolchain\* under either Linux or
Cygwin (with native Windows GNU tools or Cygwin-based GNU tools).

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain (CodeSourcery or devkitARM), or 4) Native Windows. A
DIY toolchain for inux or Cygwin is provided by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package.

Microchip SAM3X
---------------

There are two SAM3X boards supported:

#. The `Arduino <http://arduino.cc//>`__ Due development board that
   features the ATSAM3X8E MCU running at 84MHz. See the `Arduino
   Due <http://arduino.cc/en/Main/arduinoBoardDue>`__ page for more
   information.
#. The Mikroelektronika `Flip&Click
   SAM3X <https://www.mikroe.com/flip-n-click-sam3x>`__ development
   board. This board is an Arduino Due *work-alike* with additional
   support for 4 mikroBUS Click boards.

**Development Environments:** See the Microchip SAM3U discussion
`above. <#at91sam3u>`__

Microchip SAM4L
---------------

This port uses the Microchip SAM4L Xplained Pro development
board. This board features the ATSAM4LC4C MCU running at 48MHz with
256KB of FLASH and 32KB of internal SRAM.

**STATUS:** As of this writing, the basic port is code complete and a
fully verified configuration exists for the NuttShell
:ref:`NSH <nsh>`). The first
fully functional SAM4L Xplained Pro port was released in NuttX-6.28.
Support for the SAM4L Xplained modules was added in NuttX-6.29:

-  Support for the SPI-based SD card on the I/O1 module.
-  Driver for the LED1 segment LCD module.
-  Support for the UG-2832HSWEG04 OLED on the SAM4L Xplained Pro's OLED1
   module

Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/sam34/sam4l-xplained/README.txt>`__
file for further information.

**Memory Usage**. The ATSAM4LC4C comes in a 100-pin package and has
256KB FLASH and 32KB of SRAM. Below is the current memory usage for the
NSH configuration (June 9, 2013). This is *not* a minimal
implementation, but a full-featured NSH configuration.

Static memory usage can be shown with ``size`` command:

NuttX, the NSH application, and GCC libraries use 42.6KB of FLASH
leaving 213.4B of FLASH (83.4%) free from additional application
development. Static SRAM usage is about 2.3KB (<7%) and leaves 29.7KB
(92.7%) available for heap at runtime.

SRAM usage at run-time can be shown with the NSH ``free`` command. This
runtime memory usage includes the static memory usage *plus* all dynamic
memory allocation for things like stacks and I/O buffers:

You can see that 22.8KB (71.1%) of the SRAM heap is still available for
further application development while NSH is running.

Microchip SAM4CM
----------------

General architectural support was provided for SAM4CM
family in NuttX 7.3 This was *architecture-only* support, meaning that
support for the boards with these chips is available, but no support for
any publicly available boards was included. The SAM4CM port should be
compatible with most of the SAM3/4 drivers (like HSMCI, DMAC, etc.) but
those have not be verified on hardware as of this writing. This support
was contributed in part by Max Neklyudov.

**Microchip SAM4CMP-DB**. Support for the SAM4CMP-DB board was contributed
to NuttX by Masayuki Ishikawa in NuttX-7.19. The SAM4CM is a dual-CPU
part and SMP was included for the ARMv7-M and SAM3/4 families. The
SAM4CMP-DB board support includes an NSH configuration that operates in
an SMP configuration. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/sam34/sam4cmp-db/README.txt>`__
file for further information.

Microchip SAM4E
---------------

General architectural support was provided for the SAM4E
family in NuttX 6.32. This was *architecture-only* support, meaning that
support for the boards with these chips is available, but no support for
any publicly available boards was included. This support was contributed
in part by Mitko.

**Microchip SAM4E-EK**. Board support was added for the SAM4E-EK development
board in NuttX 7.1. A fully functional NuttShell (NSH) configuration is
available (see :ref:`NSH <nsh>`). That NSH
configuration includes networking support and support for an AT25 Serial
FLASH file system.

Microchip SAM4S
---------------

There are ports to two Microchip SAM4S board:

-  There is a port the Microchip SAM4S Xplained development board. This
   board features the ATSAM4S16 MCU running at 120MHz with 1MB of FLASH
   and 128KB of internal SRAM.

-  There is also a port to the Microchip SAM4S Xplained *Pro* development
   board. This board features the ATSAM4S32C MCU running at 120MHz with
   2MB of FLASH and 160KB of internal SRAM.

Microchip SAM4E. General architectural support was provided for the SAM4E
family in NuttX 6.32. This was *architecture-only* support, meaning that
support for the boards with these chips is available, but no support for
any publicly available boards was included. This support was contributed
in part by Mitko.

**Microchip SAM4E-EK**. Board support was added for the SAM4E-EK development
board in NuttX 7.1. A fully functional NuttShell (NSH) configuration is
available (see :ref:`NSH <nsh>`). That NSH
configuration includes networking support and support for an AT25 Serial
FLASH file system.

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU Cortex-M3 or 4 toolchain, 3) Cygwin/MSYS
with Windows native GNU Cortex-M3 or M4 toolchain (CodeSourcery or
devkitARM), or 4) Native Windows. A DIY toolchain for Linux or Cygwin is
provided by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package.

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
