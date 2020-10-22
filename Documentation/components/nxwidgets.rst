.. _nxwidgets:

=========
NxWidgets
=========

In order to better support NuttX based platforms, a special graphical
userinterface has been created called NXWidgets. NXWidgets is written in
C++ and integrates seamlessly with the NuttX :ref:`NX graphics
subsystem <nxgraphics>` in order to provide graphic
objects, or "widgets," in the NX Graphics Subsystem

Some of the features of NXWidgets include:

-  **Conservative C++**. NXWidgets is written entirely in C++ but using
   only selected "embedded friendly" C++ constructs that are fully
   supported under NuttX. No additional C++ support libraries are
   required.
-  **NX Integration**. NXWidgets integrate seamlessly with the
   :ref:`NX graphics subsystem <nxgraphics>`. Think of the X
   server under Linux … the NX graphics system is like a tiny X server
   that provides windowing under NuttX. By adding NXWidgets, you can
   support graphics objects like buttons and text boxes in the NX
   windows and toolbars.
-  **Small Footprint**. NXWidgets is tailored for use MCUs in embedded
   applications. It is ideally suited for mid- and upper-range of most
   MCU families. A complete NXWidgets is possible in as little as 40K of
   FLASH and maybe 4K of SRAM.
-  **Output Devices**. NXWidgets will work on the high-end frame buffer
   devices as well as on LCDs connected via serial or parallel ports to
   a small MCU.
-  **Input Devices**. NXWidgets will accept position and selection
   inputs from a mouse or a touchscreen. It will also support character
   input from a keyboard such as a USB keyboard. NXWidgets supports on
   very special widget called CKeypad that will provide keyboard input
   via an on-screen keypad that can be operated via mouse or touchscreen
   inputs.
-  **Many Graphic Objects**. Some of the graphic objects supported by
   NXWidgets include labels, buttons, text boxes, button arrays, check
   boxes, cycle buttons, images, sliders, scrollable list boxes,
   progress bars, and more.
-  **DOxygen Documentation** DOxygen documentation is available.

Note: Many of the fundamental classed in NxWidgets derive from the
Antony Dzeryn's "Woopsi" project which also has a
BSD style license. See the COPYING file for details.

NXWidgets Doxygen Documentation
===============================

.. todo::
   NXWidgets supports building HTML documentation via Doxygen. We should
   integrate this into the Sphinx documentation build.

Thanks go to Jose Pablo Carballo for contributing this!
