=========
makerlisp
=========

This port use the MakerLisp machine based on an eZ80F091 ez80Acclaim!
Microcontroller, and the Zilog ZDS-II Windows command line tools.  The
development environment is Cygwin under Windows. A Windows native
development environment is available but has not been verified.

Configurations
==============

nsh_flash
---------

``nsh.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``nsh.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``nsh_flash.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/makerlisp_ram.ztgt``.

``nsh_ram.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/makerlisp_flash.ztgt``.

nsh_ram
-------

``nsh.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``nsh.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``nsh_flash.ztgt`` is the target file that accompanies the project files.  This
  one is identical to boards/scripts/makerlisp_ram.ztgt.

``nsh_ram.ztgt``
  is the target file that accompanies the project files.  This
  one is identical to boards/scripts/makerlisp_flash.ztgt.

sdboot
------

``sdboot.zdsproj`` is a simple ZDS-II project that will allow you
  to use the ZDS-II debugger.

``sdboot.zfpproj`` is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``sdboot_flash.ztgt`` is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/makerlisp_ram.ztgt``.

``sdboot_ram.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/makerlisp_flash.ztgt``.
