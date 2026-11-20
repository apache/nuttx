===================
``checkkconfig.py``
===================

``checkkconfig.py`` is a Python script that simulates the effects of modifying a CONFIG item.
It can be used to check whether my config changes are what I expected.

Help message::

  $ tools/checkkconfig.py -h
  usage: checkkconfig.py [-h] [-o OUT_DIR] -f FILE
                         (-s CONFIG [VALUE ...] | -d DIFF)

  Simulate the effects of modifying CONFIG items in NuttX

  options:
    -h, --help            show this help message and exit
    -o OUT_DIR, --out_dir OUT_DIR
                          Binary dir for CMake configured
    -f FILE, --file FILE  Path to the input defconfig file
    -s CONFIG [VALUE ...], --set CONFIG [VALUE ...]
                          Analyze multiple changes: CONFIG_NAME1 y/m/n
                          CONFIG_NAME2 y/m/n ...
    -d DIFF, --diff DIFF  Analyze changes from diff file

  example: ./tools/checkkconfig.py -f defconfig -s ELF n

  outputs:
  Change report for config changes
  Config Option                            Old                  New
  ----------------------------------------------------------------------
  BINFMT_LOADABLE                          y                    n
  ELF                                      y                    n
  ELF_STACKSIZE                            8192                 <unset>
  LIBC_ARCH_ELF                            y                    n
  LIBC_MODLIB                              y                    n
  MODLIB_ALIGN_LOG2                        2                    <unset>
  MODLIB_BUFFERINCR                        32                   <unset>
  MODLIB_BUFFERSIZE                        32                   <unset>
  MODLIB_MAXDEPEND                         2                    <unset>
  MODLIB_RELOCATION_BUFFERCOUNT            256                  <unset>
  MODLIB_SYMBOL_CACHECOUNT                 256                  <unset>

As we can see, we can clearly know that
if I turn off ELF in defconfig at this time,
it will bring about the following configuration linkage changes

It can also parse diff files, which can be used to check the changes of multiple
configs.

``-s``/``--set`` accepts several ``CONFIG VALUE`` pairs, so the combined effect of
multiple changes can be checked at once.  A value other than ``y``, ``m`` or
``n`` is set as is, for example a number or a string::

  $ ./tools/checkkconfig.py -f defconfig -s ELF y TASK_NAME_SIZE 16

CMake builds keep the configuration in the build directory instead of the
top-level NuttX directory.  Pass that directory with ``-o``/``--out_dir`` to
analyze a CMake configuration::

  $ cmake -B build -DBOARD_CONFIG=sim:nsh -GNinja
  $ ./tools/checkkconfig.py -o build -f build/.config -s ELF y
