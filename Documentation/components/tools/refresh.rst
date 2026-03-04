==============
``refresh.sh``
==============

.. note::

   This script with ``--silent`` is really obsolete. The silent option really
   adds default values. However, as of 217-07-09, defconfig files are retained
   in a compressed format, i.e., with default values removed.  So the
   ``--silent`` option will accomplish nothing. Without ``--silent``, you will
   have the opportunity over override the default value from the command line
   and, in that case, the script may still have some minimal value.

This is a bash script that automates refreshing of board default configuration
(defconfig) files. It does not do anything special that you cannot do manually,
but is useful for updating dozens of configuration files. It is also used in the
NuttX CI process.

Configuration files have to be updated because over time, the configuration
settings change; new configurations are added and new dependencies are added.
So an old configuration file may not be usable anymore until it is refreshed.

Help is also available:

.. code:: console

   $ tools/refresh.sh --help
   tools/refresh.sh is a tool for refreshing board configurations

   USAGE: tools/refresh.sh [options] <board>:<config>+

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

     Note1: all configurations are refreshed if <board>:<config> is replaced with "all" keyword
     Note2: all configurations of arch XYZ are refreshed if "arch:<namearch>" is passed
     Note3: all configurations of chip XYZ are refreshed if "chip:<chipname>" is passed
     Note4: all configurations of board XYZ are refreshed if "board:<boardname>" is passed

The steps to refresh the file taken by ``refresh.sh`` are:

1. Make ``tools/cmpconfig`` if it is not already built.

2. Copy the defconfig file to the top-level NuttX directory as ``.config``
   (being careful to save any previous ``.config`` file that you might want to
   keep!).

3. Execute ``make` oldconfig` to update the configuration. ``make oldconfig``
   will prompt you for each change in the configuration that requires that you
   make some decision. With the ``--silent`` option, the script will use ``make
   oldefconfig`` instead and you won't have to answer any questions; the refresh
   will simply accept the default value for any new configuration settings.

4. Then it runs ``tools/cmpconfig`` to show the real differences between the
   configuration files.  Configuration files are complex and things can move
   around so a simple 'diff' between two configuration files is often not
   useful.  But tools/cmpconfig will show only the meaningful differences
   between the two configuration files.

5. It will edit the .config file to comment out the setting of the
   ``CONFIG_APPS_DIR=`` setting. This setting should not be in checked-in
   defconfig files because the actually must be determined at the next time that
   the configuration is installed.

6. Finally, the refreshed defconfig file is copied back in place where it can be
   committed with the next set of difference to the command line. If you select
   the ``--silent`` option, this file copy will occur automatically. Otherwise,
   refresh.sh will prompt you first to avoid overwriting the defconfig file with
   changes that you may not want.

Usage examples:

Update all boards without verbose output:

.. code:: console

   $ ./tools/refresh.sh --silent --defaults all

Update all boards and configs from `arm` architecture:

.. code:: console

   $ ./tools/refresh.sh --silent arch:arm

Update all boards from ``stm32f7`` chip family:

.. code:: console

   $ ./tools/refresh.sh --silent chip:stm32f7

Update all configs from ``stm32f103-minimum`` board:

.. code:: console

   $ ./tools/refresh.sh --silent board:stm32f103-minimum

Update only the `.nsh.` config from stm32f103-minimum board:

.. code:: console

   $ ./tools/refresh.sh --silent stm32f103-minimum:nsh
