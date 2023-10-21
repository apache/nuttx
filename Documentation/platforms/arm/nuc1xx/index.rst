==============
nuvoTon NUC120
==============

This is a port of NuttX to the nuvoTon NuTiny-SDK-NUC120
that features the NUC120LE3AN MCU.

**STATUS**. Initial support for the NUC120 was released in NuttX-6.26.
This initial support is very minimal: There is a NuttShell
(:ref:`NSH <nsh>`) configuration that might be the basis for an
application development. As of this writing, more device drivers are
needed to make this a more complete port. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/nuc1xx/nutiny-nuc120/README.txt>`__
file for further information.

**Memory Usage**. For a full-featured RTOS such as NuttX, providing
support in a usable and meaningful way within the tiny memories of the
NUC120 demonstrates the scalability of NuttX. The NUC120LE2AN comes in a
48-pin package and has 128KB FLASH and 16KB of SRAM. When running the
NSH configuration (itself a full up application), there is still more
than 90KB of FLASH and 10KB or SRAM available for further application
development).

Static memory usage can be shown with ``size`` command:

NuttX, the NSH application, and GCC libraries use 34.2KB of FLASH
leaving 93.8KB of FLASH (72%) free from additional application
development. Static SRAM usage is about 1.2KB (<4%) and leaves 14.8KB
(86%) available for heap at runtime. SRAM usage at run-time can be shown
with the NSH ``free`` command:

You can see that 10.0KB (62%) is available for further application
development.

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain, or 4) Native Windows. A DIY toolchain for Linux or
Cygwin is provided by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package.
