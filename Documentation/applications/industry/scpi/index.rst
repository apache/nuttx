====================================
``scpi`` SCPI instrument side parser
====================================

Overview
========

The ``scpi`` application integrates the
`scpi-parser <https://github.com/j123b567/scpi-parser>`_ library with the
NuttX application build system. SCPI, the Standard Commands for Programmable
Instruments, is commonly used by test and measurement equipment to parse
instrument commands and responses.

The application provides the SCPI parser library for NuttX applications and can
optionally build the upstream interactive demo program.

Source Location
===============

The NuttX application wrapper is located in ``apps/industry/scpi``. It
downloads and builds ``scpi-parser`` version 2.2 unless an unpacked
``scpi-parser`` source tree is already present in that directory.

Basic Configuration
===================

Enable the parser with:

- ``CONFIG_SCPI_PARSER`` - Build the SCPI instrument-side parser library.

Optional demo support is controlled by:

- ``CONFIG_SCPI_PARSER_DEMO`` - Build the SCPI parser interactive demo program.
- ``CONFIG_SCPI_PARSER_DEMO_PRIORITY`` - Demo task priority.
- ``CONFIG_SCPI_PARSER_DEMO_STACKSIZE`` - Demo task stack size.

Unit Parser Configuration
=========================

``scpi-parser`` can include or exclude optional unit groups at compile time.
NuttX exposes these as Kconfig options and maps each option to the matching
upstream ``USE_UNITS_*`` compile-time define.

The following unit groups are configurable:

- ``CONFIG_SCPI_PARSER_UNITS_IMPERIAL`` - Imperial units.
- ``CONFIG_SCPI_PARSER_UNITS_ANGLE`` - Angle units.
- ``CONFIG_SCPI_PARSER_UNITS_PARTICLES`` - Particle units.
- ``CONFIG_SCPI_PARSER_UNITS_DISTANCE`` - Distance units.
- ``CONFIG_SCPI_PARSER_UNITS_MAGNETIC`` - Magnetic units.
- ``CONFIG_SCPI_PARSER_UNITS_LIGHT`` - Light units.
- ``CONFIG_SCPI_PARSER_UNITS_ENERGY_FORCE_MASS`` - Energy, force, and mass
  units.
- ``CONFIG_SCPI_PARSER_UNITS_TIME`` - Time units.
- ``CONFIG_SCPI_PARSER_UNITS_TEMPERATURE`` - Temperature units.
- ``CONFIG_SCPI_PARSER_UNITS_RATIO`` - Ratio units.
- ``CONFIG_SCPI_PARSER_UNITS_POWER`` - Power units.
- ``CONFIG_SCPI_PARSER_UNITS_FREQUENCY`` - Frequency units.
- ``CONFIG_SCPI_PARSER_UNITS_ELECTRIC`` - Electric units.
- ``CONFIG_SCPI_PARSER_UNITS_ELECTRIC_CHARGE_CONDUCTANCE`` - Electric charge
  and conductance units.

By default, ``CONFIG_SCPI_PARSER_UNITS_POWER``,
``CONFIG_SCPI_PARSER_UNITS_FREQUENCY``, and
``CONFIG_SCPI_PARSER_UNITS_ELECTRIC`` are enabled. The other optional unit
groups are disabled by default and may be enabled individually.

For example, enabling ``CONFIG_SCPI_PARSER_UNITS_TIME`` passes
``USE_UNITS_TIME=1`` to ``scpi-parser`` during compilation.

Example Configuration
=====================

A minimal parser configuration can enable only the library:

.. code-block:: text

   CONFIG_SCPI_PARSER=y

To build the parser with time-unit support:

.. code-block:: text

   CONFIG_SCPI_PARSER=y
   CONFIG_SCPI_PARSER_UNITS_TIME=y

To include the interactive demo program:

.. code-block:: text

   CONFIG_SCPI_PARSER=y
   CONFIG_SCPI_PARSER_DEMO=y
