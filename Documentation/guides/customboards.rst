====================
Custom Boards How-To
====================

As explained in :doc:`../quickstart/configuring`, supported boards (also known
as "in-tree" boards) are configured using a standard syntax:

    .. code-block:: console

      $ cd nuttx
      $ ./tools/configure.sh -l board-name:config-name
        Copy files
        Select CONFIG_HOST_LINUX=y
        Refreshing...

Sometimes it is not appropriate, or not wanted, to add a new or custom board to
the NuttX boards tree itself. If so, the board can be defined out-of-tree in a
custom directory and still be built easily.

Add a Custom Board
==================

The same set of files as provided for in-tree boards is required (i.e. configs,
Kconfig, scripts, etc.) but these can be placed in a directory of your choice.

In this example, the files are assumed to exist in:
 ``../nuttx/CustomBoards/MyCustomBoardName``

    .. code-block:: console

      $pwd
      /home/nuttx/nuttx
      $ ls -1 ../CustomBoards/MyCustomBoardName
      configs
      helpers
      include
      Kconfig
      scripts
      $ ls ../CustomBoards/MyCustomBoardName/configs
      nsh
      MyCustomConfig
      $


To build the custom board, the syntax is slightly different to in-tree boards and configs:

    .. code-block:: console

      $ .tools/configure -l ../CustomBoards/MyCustomBoardName/MyCustomConfig
      Copy files
      Select CONFIG_HOST_LINUX=y
      Refreshing...

Kconfig Settings
================

Once the board is configured, to ensure subsequent builds run correctly, there
are two Kconfig settings that need to be set. These are:

:menuselection:`Board Selection --> Custom Board Configuration --> Custom Board Name`

:menuselection:`Board Selection --> Custom Board Configuration --> Relative custom board directory`

They should be set to suit your board name and directory location.

.. Note::
   If you subsequently run a ``make distclean`` operation, then these settings will be lost.
   They should be added back before building, and/or before running ``make menuconfig``.
