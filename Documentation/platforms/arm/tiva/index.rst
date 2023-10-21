=================
TI/Stellaris Tiva
=================

TI/Stellaris LM3S6432
---------------------

This is a port of NuttX to the Stellaris RDK-S2E
Reference Design Kit and the MDL-S2E Ethernet to Serial module
(contributed by Mike Smith).

TI/Stellaris LM3S6432S2E
------------------------

This port uses Serial-to-Ethernet Reference
Design Kit and has
similar support as for the other Stellaris family members. A
configuration is available for the NuttShell (NSH) (see :ref:`NSH <nsh>`). The NSH
configuration including networking support with a Telnet NSH console.
This port was contributed by Mike Smith.

TI/Stellaris LM3S6918
---------------------

This port uses the
`Micromint <http://www.micromint.com/>`__ Eagle-100 development board
with a GNU arm-nuttx-elf toolchain\* under either Linux or Cygwin.

**Development Environments:** 1) Linux with native Linux GNU toolchain,
2) Cygwin/MSYS with Cygwin GNU toolchain, 3) Cygwin/MSYS with Windows
native toolchain (CodeSourcery or devkitARM), or 4) Native Windows. A
DIY toolchain for Linux or Cygwin is provided by the NuttX
`buildroot <https://bitbucket.org/nuttx/buildroot/downloads/>`__
package.

TI/Stellaris LM3S6965
---------------------

This port uses the Stellaris LM3S6965 Ethernet
Evaluation Kit with a GNU arm-nuttx-elf toolchain\* under either Linux
or Cygwin.

**Development Environments:** See the Eagle-100 LM3S6918 above.

TI/Stellaris LM3S8962
---------------------

This port uses the Stellaris EKC-LM3S8962
Ethernet+CAN Evaluation Kit with a GNU arm-nuttx-elf toolchain\* under
either Linux or Cygwin. Contributed by Larry Arnold.

TI/Stellaris LM3S9B92
---------------------

Architectural support for the LM3S9B92 was
contributed by Lwazi Dube in NuttX 7.28. No board support for boards
using the LM3S9B92 are currently available.

TI/Stellaris LM3S9B96
---------------------

Header file support was contributed by Tiago
Maluta for this part. Jose Pablo Rojas V. is used those header file
changes to port NuttX to the TI/Stellaris EKK-LM3S9B96. That port was
available in the NuttX-6.20 release. Refer to the NuttX board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/tiva/ekk-lm3s9b96/README.txt>`__
file for further information.

TI/SimpleLink CC13x0
--------------------

Basic, unverified architectural support for the
CC13x0 was added in NuttX-7.28. This is a work in progress and, with any
luck, a fully verified port will be available in NuttX-7.29.

TI/Tiva TM4C123G
----------------

This port uses the Tiva C Series TM4C123G LaunchPad
Evaluation Kit
`(EK-TM4C123GXL) <http://www.ti.com/tool/ek-tm4c123gxl>`__.

**TI Tiva TM4C123H**. Architectural support for the Tiva TM4C123AH6PM
was contributed in NuttX-8.1 by Nathan Hartman.

**STATUS:**

-  **NuttX-7.1**. Initial architectural support for the EK-TM4C123GXL
   was implemented and was released in NuttX 7.1. Basic board support
   the EK-TM4C123GXL was also included in that release but was not fully
   tested. This basic board support included a configuration for the
   NuttShell
   :ref:`NSH <nsh>`).
-  **NuttX-7.2**. The fully verified port to the EK-TM4C123GXL was
   provided in NuttX-7.2.
-  **NuttX-7.7**. An I2C driver was added in NuttX-7.7.
-  **NuttX-8.1**. Along with TM4C123AH6PM support, Nathan Hartman also
   reinstated and extended the Tiva Quadrature Encoder driver.

TI/Tiva TM4C1294
----------------

This port uses the TI Tiva C Series TM4C1294 Connected
LaunchPad `(EK-TM4C1294XL) <http://www.ti.com/tool/ek-tm4c1294xl>`__.

**STATUS:**

-  Support for the EK-TM4C1294XL was contributed by Frank Sautter and
   was released in NuttX 7.9. This basic board support included a
   configuration for the NuttShell
   :ref:`NSH <nsh>`) and a
   configuration for testing IPv6. See drivers for the `TI Tiva
   TM4C129X <#titm4c129x>`__.
-  FLASH and EEPROM drivers from Shirshak Sengupta were included in
   NuttX-7.25.

Refer to the EK-TM4C1294XL board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/tiva/tm4c1294-launchpad/README.txt>`__
file for more detailed information about this port.

TI/Tiva TM4C129E
----------------

This port uses the TI Tiva C Series TM4C129E Crypto Connected
LaunchPad `(EK-TM4C129EXL) <https://www.ti.com/tool/EK-TM4C129EXL>`__.

**STATUS:**

-  Support for the EK-TM4C129EXL is based on support for the similar
   EK-TM4C1294XL. This basic board support includes a configuration
   for the NuttShell :ref:`NSH <nsh>`), a configuration for testing
   IPv6, and a configuration for testing the RTOS using the ostest
   example in the NuttX apps repository.

Refer to the EK-TM4C129EXL board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/tiva/tm4c129e-launchpad/README.txt>`__
file for more detailed information about this port.

TI/Tiva TM4C129X
----------------

This port uses the TI Tiva C Series TM4C129X Connected
Development Kit `(DK-TM4C129X) <http://www.ti.com/tool/dk-tm4c129x>`__.

**STATUS:**

-  A mature port to the DK-TM4C129X was implemented and was released in
   NuttX 7.7.
-  At the initial release, verified drivers were available for Ethernet
   interface, I2C, and timers as well as board LEDs and push buttons.
   Other Tiva/Stellaris drivers should port to the TM4C129X without
   major difficulty.
-  This board supports included two configurations for the NuttShell
   (:ref:`NSH <nsh>`). Both
   are networked enabled: One configured to support IPv4 and one
   configured to supported IPv6. Instructions are included in the board
   `README <https://github.com/apache/nuttx/blob/master/boards/arm/tiva/dk-tm4c129x/README.txt>`__
   file for configuring both IPv4 and IPv6 simultaneously.
-  Tiva PWM and Quadrature Encoder drivers were contributed to NuttX in
   7.18 by Young.

Refer to the DK-TM4C129X board
`README <https://github.com/apache/nuttx/blob/master/boards/arm/tiva/dk-tm4c129x/README.txt>`__
file for more detailed information about this port.

TI/SimpleLink CC13x2
--------------------

Basic, unverified architectural support for the
CC13x2 was added in NuttX-7.28. Fragmentary support for very similar
CC26x2 family is included. This is a work in progress and, with any
luck, a fully verified port will be available in NuttX-7.29. It is
currently code complete (minus some ROM *DriverLib* hooks) but untested.

**TI LaunchXL-CC1312R1**. Basic board support for the TI
LaunchXL-CC1312R1 board is in place. Board bring-up, however, cannot be
done until the the basic CC13x2 architecture support is complete,
hopefully in NuttX-7.29.

TI/Stellaris LM4F120x
---------------------

This port uses the TI Stellaris LM4F120 LaunchPad.
Jose Pablo Carballo and I are doing this port.

- TI/Tiva TM4C123G
- TI/Tiva TM4C1294
- TI/Tiva TM4C129E
- TI/Tiva TM4C129X
- TI/SimpleLink CC13x2
