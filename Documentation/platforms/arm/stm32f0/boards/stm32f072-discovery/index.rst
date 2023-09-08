======================
ST STM32F072-DISCOVERY
======================

STATUS
======

05/17: The basic NSH configuration is functional and shows that there is
3-4KB of free heap space.  However, attempts to extend this have
failed.  I suspect that 8KB of SRAM is insufficient to do much
with the existing NSH configuration.  Perhaps some fine tuning
an improve this situation but at this point, I think this board
is only useful for the initial STM32 F0 bring-up, perhaps for
embedded solutions that do not use NSH and for general
experimentation.

There is also support for the Nucleo boards with the STM32 F072
and F092 MCUs.  Those ports do not suffer from these problems and
seem to work well in fairly complex configurations.  Apparently 8KB
is SRAM is not usable but the parts with larger 16KB and 32KB SRAMs
are better matches.
