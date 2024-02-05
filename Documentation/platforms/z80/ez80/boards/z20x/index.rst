====
z20x
====

Microcontroller.  This directory holds the port of NuttX to the z80x board
based on an ez80Acclaim! eZ80F091 microcontroller.

Configurations
==============


hello
-----

``hello.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``hello.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``hello_ram.ztgt`` is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_ram.ztgt``.

``hello_flash.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_flash.ztgt``.

nsh
---

``nsh.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``nsh.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``nsh_ram.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_ram.ztgt``.

``nsh_flash.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_flash.ztgt``.

sdboot
------

``sdboot.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``sdboot.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``sdboot_flash.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_ram.ztgt``.

``sdboot_ram.ztgt``
  is the target file that accompanies the project files.
  This one is identical to ``boards/scripts/z20x_flash.ztgt``.

w25boot
-------

``w25boot.zdsproj``
  is a simple ZDS-II project that will allow you to use the ZDS-II debugger.

``w25boot.zfpproj``
  is a simple project that will allow you to use the Smart Flash
  Programming.  NOTE:  As of this writing this project does not work, probably
  due to RAM configuration in the project.  Use ZDS-II instead as is described
  in the upper README.txt file

``w25boot_flash.ztgt``
  is the target file that accompanies the project files.
  This one is identical to boards/scripts/z20x_ram.ztgt.

``w25boot_ram.ztgt``
  is the target file that accompanies the project files.
  This one is identical to boards/scripts/z20x_flash.ztgt.
