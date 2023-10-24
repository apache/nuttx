=======================
``nxwidgets`` NXWidgets
=======================

In order to better support NuttX based platforms, a special graphical user
interface has been created called NXWidgets. NXWidgets is written in C++ and
integrates seamlessly with the NuttX NX graphics subsystem in order to provide
graphic objects, or _widgets_, in the NX Graphics Subsystem

Some of the features of NXWidgets include:

- Conservative C++

  NXWidgets is written entirely in C++ but using only selected "embedded
  friendly" C++ constructs that are fully supported under NuttX. No additional
  C++ support libraries are required.

- NX Integration

  NXWidgets integrate seamlessly with the NX graphics system. Think of the X
  server under Linux – the NX graphics system is like a tiny X server that
  provides windowing under NuttX. By adding NXWidgets, you can support graphics
  objects like buttons and text boxes in the NX windows and toolbars.

- Small Footprint

  NXWidgets is tailored for use MCUs in embedded applications. It is ideally
  suited for mid- and upper-range of most MCU families. A complete NXWidgets is
  possible in as little as 40Kb of FLASH and maybe 4Kb of SRAM.

- Output Devices

  NXWidgets will work on the high-end frame buffer devices as well as on LCDs
  connected via serial or parallel ports to a small MCU.

- Input Devices

  NXWidgets will accept position and selection inputs from a mouse or a
  touchscreen. It will also support character input from a keyboard such as a
  USB keyboard. NXWidgets supports on very special widget called ``CKeypad`` that
  will provide keyboard input via an on-screen keypad that can be operated via
  mouse or touchscreen inputs.

- Many Graphic Objects

  Some of the graphic objects supported by NXWidgets include labels, buttons,
  text boxes, button arrays, check boxes, cycle buttons, images, sliders,
  scrollable list boxes, progress bars, and more.

**Note**: Many of the fundamental classed in NxWidgets derive from the Antony
Dzeryn's _Woopsi_ project: http://woopsi.org/ which also has a BSD style
license. See the ``COPYING`` file for details.

Directory Structure
-------------------

- ``Kconfig``

  This is a ``Kconfig`` file that should be provided at ``apps/NxWidgets/Kconfig``.
  When copied to that location, it will be used by the NuttX configuration
  systems to configure settings for NxWidgets and NxWM

- ``nxwidgets``

  The source code, header files, and build environment for NxWidgets is provided
  in this directory.

- ``UnitTests``

  Provides a collection of unit-level tests for many of the individual widgets
  provided by ``nxwidgets``.

Doxygen
-------

Installing the necessary packages in Ubuntu
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Install the following packages::

   $ sudo aptitude install doxygen doxygen-doc doxygen-gui dot2tex graphviz

2. (Optional) Install Doxygen from the latest sourcode.

   The Ubuntu package is outdated. The newer the version of Doxygen, the better
   the documentation looks.

   Place yourself in some temporary folder where you can download the source,
   and run [1]::

     $ svn co https://doxygen.svn.sourceforge.net/svnroot/doxygen/trunk doxygen-svn
     $ cd doxygen-svn
     $ ./configure
     $ make
     $ make install

Generating documentation
~~~~~~~~~~~~~~~~~~~~~~~~

Two ways described here:

1. Use the provided ``gendoc.sh`` script::

     trunk/NXWidgets/Doxygen/gendoc.sh

   The script only needs the argument to the absolute path where to place the
   generated documentation. I.e.::

     $ cd /path/to/nuttx/trunk/NXWidgets/Doxygen/
     $ mkdir doc
     $ ./gendoc.sh $PWD/doc

2. Using the ``Doxyfile`` directly:

   The file ``Doxyfile`` contains the configuration of the Doxygen settings for
   the run, edit only if necessary.

   To generate the documentation type::

     $ cd /path/to/nuttx/trunk/NXWidgets/Doxygen/
     $ doxygen Doxyfile

References
~~~~~~~~~~

[1] http://www.stack.nl/~dimitri/doxygen/download.html


Unit Tests
----------

Installing and Building the Unit Tests
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Setup NuttX

   1. Configure NuttX

      Configure NuttX to run one of the target configurations. For example,
      let's assume that you are using the ``sim/nsh2`` configuration. The
      ``sim/nsh2`` configuration was specially created for use NXWidgets on the
      simulation platform. A similar, special configuration ``stm3210e-eval/nsh2``
      is also for the ``STM3210E-EVAL`` available. However, the unit test can be
      run on other configurations (see steps d and e below).

      **Note**: There are some other special configurationsrecommended for
      unit-leveling testing of NxWM because the configuration is more complex in
      that case. These are:

      1) ``sim/nxwmm``, or the simulated platform (no touchscreen), and
      2) ``stm3240g-evel``, for the ``STM3240G-EVAL`` board (with the STMPE11
          touchscreen)

      We will assume the ``sim/nsh2`` configuration in this discussion. The
      ``sim/nsh2`` configuration is installed as follows::

        cd <nuttx-directory-path>
        make distclean
        tools/configure.sh sim:nsh2

      Where:

      ``<nuttx-directory-path>`` is the full, absolute path to the NuttX build
      directory

      If you are using the ``sim/nsh2`` or ``stm3210e-eval`` configurations, then
      skip to step 2 (Hmmm.. better check 1d) too).

      There may be certain requirements for the configuration that you select...
      for example, certain widget tests may require touchscreen support or
      special font selections. These test-specific requirements are addressed
      below under "Unit Test Directories"

   2. Enable C++ Support

      If you are not using the ``sim/nsh2`` or ``stm3210e-eval``, you will need to
      add the following definitions to the NuttX configuration at
      ``nuttx/.config`` to enable C++ support::

        CONFIG_HAVE_CXX=y

      Check first, some configurations already have C++ support enabled (As of
      this writing **ONLY** the ``sim/nsh2`` and ``stm321-e-eval`` configurations
      have C++ support pre-enabled).

   3. Enable Debug Options

      If you are running on a simulated target, then you might also want to
      enable debug symbols::

        CONFIG_DEBUG_SYMBOLS=y

      Then you can run the simulation using GDB or DDD which is a very powerful
      debugging environment!

   4. Special configuration requirements for the nxwm unit test::

        CONFIG_NXTERM=y

   5. Other ``.config`` file changes – NSH configurations only.

      If the configuration that you are using supports NSH and NSH built-in
      tasks then all is well. If it is an NSH configuration, then you will have
      to define the following in your ``nuttx/.config`` file as well (if it is not
      already defined)::

        CONFIG_NSH_BUILTIN_APPS=y

      ``sim/nsh2`` and ``stm3210e-eval/nsh2`` already has this setting. You do not
      need to change anything further in the ``nuttx/.config`` file if you are
      using either of these configurations.

   6. Other ``.config`` file changes – NON-NSH configurations only.

      Entry Point. You will need to set the entry point in the .config file. For
      NSH configurations, the entry point will always be ``nsh_main`` and you will
      see that setting like::

        CONFIG_INIT_ENTRYPOINT="nsh_main"

      If you are not using in NSH, then each unit test has a unique entry point.
      That entry point is the name of the unit test directory in all lower case
      plus the suffix ``_main``. So, for example, the correct entry for the
      ``UnitTests/CButton`` would be::

        CONFIG_INIT_ENTRYPOINT="cbutton_main"

      And the correct entry point for ``UnitTests/nxwm`` would be::

        CONFIG_INIT_ENTRYPOINT="nxwm_main"

      etc.

      For non-NSH configurations (such as the ``sim/touchscreen``) you will have
      to remove the configuration setting that provided the ``main`` function so
      that you use the ``main`` in the unit test code instead. So, for example,
      with the ``sim/touchscreen`` configuration you need to remove the following
      from the NuttX configuration file (``.config``)::

        CONFIG_EXAMPLES_TOUSCHCREEN=y  ## REMOVE (provided "tc_main")

2. Adjust the Stack Size

   If using an simulation configuration (like ``sim/nsh2``) and your unit test
   uses X11 as its display device, then you would have to increase the size of
   unit test stack as described below under "Stack Size Issues with the X11
   Simulation".

3. Build NuttX including the unit test and the NXWidgets library::

     cd <nuttx-directory-path>
     . ./setenv.sh
     make

Work-Arounds
~~~~~~~~~~~~

Build Issues
............

1. I have seen this error on Cygwin building C++ code::

     LD:  nuttx.rel
     ld: skipping incompatible /home/patacongo/projects/nuttx/nuttx/trunk/nuttx/libxx//liblibxx.a when searching for -llibxx
     ld: cannot find -llibxx

   The problem seems to be caused because ``gcc`` build code for 32-bit mode and
   ``g++`` builds code for 64-bit mode. Add the ``-m32`` option to the ``g++`` command
   line seems to fix the problem. In ``Make.defs``::

     CXXFLAGS = -m32 $(ARCHWARNINGSXX) $(ARCHOPTIMIZATION) \
                $(ARCHCXXFLAGS) $(ARCHINCLUDESXX) $(ARCHDEFINES) $(EXTRADEFINES) -pipe

2. Stack Size Issues with the X11 Simulation

   When you run the NuttX simulation, it uses stacks allocated by NuttX from the
   NuttX heap. The memory management model is exactly the same in the simulation
   as it is real, target system. This is good because this produces a higher
   fidelity simulation.

   However, when the simulation calls into Linux/Cygwin libraries, it will still
   use these small simulation stacks. This happens, for example, when you call
   into the system to get and put characters to the console window or when you
   make x11 calls into the system. The programming model within those libraries
   will assume a Linux/Cygwin environment where the stack size grows dynamically

   As a consequence, those system libraries may allocate large data structures
   on the stack and overflow the small NuttX stacks. X11, in particular,
   requires large stacks. If you are using X11 in the simulation, make sure that
   you set aside a "lot" of stack for the X11 system calls (maybe 8 or 16Kb).
   The stack size for the thread that begins with user start is controlled by
   the configuration setting ``CONFIG_INIT_STACKSIZE``; you may need to
   increase this value to larger number to survive the X11 system calls.

   If you are running X11 applications as NSH add-on programs, then the stack
   size of the add-on program is controlled in another way. Here are the steps
   for increasing the stack size in that case::

     cd ../apps/namedapps  # Go to the namedapps directory
     vi namedapps_list.h   # Edit this file and increase the stack size of the add-on
     rm .built *.o         # This will force the namedapps logic to rebuild

Unit Tests Directories
~~~~~~~~~~~~~~~~~~~~~~

The following provide simple unit tests for each of the NXWidgets. In addition,
these unit tests provide examples for the use of each widget type.

- ``CButton``

  - Exercises the ``CButton`` widget.
  - Depends on ``CLabel``.

- ``CButtonArray``

  - Exercises the ``CButtonArray`` widget.

- ``CCheckBox``

  - Exercises the ``CCheckBox`` widget.
  - Depends on ``CLabel`` and ``CButton``.

- ``CGlyphButton``

  - Exercises the ``CGlyphButton`` widget.
  - Depends on ``CLabel`` and ``CButton``.

- ``CImage``

  - Exercises the ``CImage`` widget.

- ``CLabel``

  - Exercises the ``CLabel`` widget.

- ``CProgressBar``

  - Exercises the ``CProgressBar`` widget.

- ``CRadioButton``

  - Exercises the ``CRadioButton`` and ``CRadioButtonGroup`` widgets.
  - Depends on ``CLabel`` and ``CButton``.

- ``CScrollBarHorizontal``

  - Exercises the ``ScrollbarHorizontal``.
  - Depends on ``CSliderHorizontal`` and ``CGlyphButton``.

- ``CScrollBarVertical``

  - Exercises the ``ScrollbarHorizontal``.
  - Depends on ``CSliderVertical`` and ``CGlyphButton``.

- ``CSliderHorizontal``

  - Exercises the ``CSliderHorizontal``.
  - Depends on ``CSliderHorizontalGrip``.

- ``CSliderVertical``

  - Exercises the ``CSliderVertical``.
  - Depends on ``CSliderVerticalGrip``.

- ``CTextBox``

  - Exercises the ``CTextBox`` widget.
  - Depends on ``CLabel``.
