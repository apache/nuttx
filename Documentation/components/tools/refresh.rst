==============
``refresh.sh``
==============

.. note::

   This script with ``--silent`` is obsolete.

   The silent option adds default values, however, as of 2017-07-09,
   defconfig files are retained in a compressed format, i.e. with default values
   removed.  So the ``--silent`` option will accomplish nothing. Without
   ``--silent``, you will have the opportunity to override the default values
   from the command line and, in that case, the script may still have some
   minimal value.

This is a bash script that automates refreshing board default configuration
(defconfig) files. It does not do anything special that you cannot do manually,
but is useful for updating dozens of configuration files. It is also used in the
NuttX CI process.

Configuration files have to be updated because, over time, the configuration
settings change; new configurations are added and new dependencies are added.
So an old configuration file may not be usable anymore until it is refreshed.

Help is also available:

.. code:: console

   $ tools/refresh.sh --help
   tools/refresh.sh is a tool for refreshing board configurations

   USAGE: tools/refresh.sh [options] (<board>:<config>|<path/to/config/folder>)+

   Where [options] include:
     --debug
        Enable script debug
     --silent
        Update board configuration without interaction.  Implies --defaults.
        Assumes no prompt for save.  Use --silent --prompt to prompt before saving.
     --prompt
        Prompt before updating and overwriting the defconfig file.  Default is to
        prompt unless --silent
     --defaults
        Do not prompt for new default selections; accept all recommended default values
     --nocopy
        Do not copy defconfig from nuttx/boards/<board>/configs to nuttx/.config
     --help
        Show this help message and exit
     <board>
        The board directory under nuttx/boards/arch/chip/
     <config>
        The board configuration directory under nuttx/boards/arch/chip/<board>/configs
     <archname>
        The architecture directory under nuttx/boards/
     <chipname>
        The chip family directory under nuttx/boards/<arch>/
     <path/to/config/folder>
        Relative or absolute path the configuration subdirectory

     Note1: all configurations are refreshed if <board>:<config> is replaced with "all" keyword
     Note2: all configurations of arch XYZ are refreshed if "arch:<namearch>" is passed
     Note3: all configurations of chip XYZ are refreshed if "chip:<chipname>" is passed
     Note4: all configurations of board XYZ are refreshed if "board:<boardname>" is passed

The steps to refresh the configurations and/or files given to ``refresh.sh`` are:

1. Copy the defconfig file to the top-level NuttX directory as ``.config`` and its
   corresponding ``Make.defs`` file (being careful to save any previous identically
   named files that you might want to keep).

2. Execute ``make oldconfig`` to update the configuration. ``make oldconfig``
   will prompt you for each change in the configuration that requires that you
   make some decision. With the ``--defaults`` option, the script will use ``make
   oldefconfig`` instead and you won't have to answer any question; the refresh
   will simply accept the default value for any new configuration setting.

3. Execute ``make savedefconfig`` to create the new defconfig file. Any setting set
   to its default value is stripped down from this file.

   This will also strip down the ``CONFIG_APPS_DIR`` and ``CONFIG_BASE_DEFCONFIG``
   settings. They should not be in checked-in defconfig files because they must be
   determined each time the configuration is installed.

4. Check for any difference between the generated defconfig and the original
   configuration file using the ``diff`` utility.

   If there are differences, it save the new configuration (with or without prompt
   depending on the ``--silent`` and ``--prompt`` options).

5. Restore the ``.config`` and ``Make.defs`` files from step 1.


Usage examples
--------------

Update all boards without verbose output:

.. code:: console

   $ ./tools/refresh.sh --defaults all

Update all boards and configs from `arm` architecture:

.. code:: console

   $ ./tools/refresh.sh --defaults arch:arm

Update all boards from ``stm32f7`` chip family:

.. code:: console

   $ ./tools/refresh.sh --defaults chip:stm32f7

Update all configs from ``stm32f103-minimum`` board:

.. code:: console

   $ ./tools/refresh.sh --defaults board:stm32f103-minimum

Update only the `nsh` config from ``stm32f103-minimum`` board:

.. code:: console

   $ ./tools/refresh.sh --defaults stm32f103-minimum:nsh

Update the `hello` config from the ``arduino-mega2560`` board and the `ostest` config
from the ``sim`` pseudo-board:

.. code:: console

  $ ./tools/refresh.sh --defaults arduino-mega2560:hello sim:ostest

Update the `nsh` config from ``stm32f103-minimum`` board, using the path to the
configuration subdirectory:

.. code:: console

  $ ./tools/refresh.sh --defaults boards/arm/stm32f1/stm32f103-minimum/configs/nsh
