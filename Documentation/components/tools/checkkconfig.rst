===================
``checkkconfig.py``
===================

``checkkconfig.py`` is a Python script that simulates the effects of modifying a CONFIG item.
It can be used to check whether my config changes are what I expected.

Help message::

  $ tools/checkkconfig.py -h
  usage: checkkconfig.py [-h] -f FILE (-s CONFIG VALUE | -d DIFF)

  optional arguments:
    -h, --help            show this help message and exit
    -f FILE, --file FILE  Path to the input defconfig file
    -s CONFIG_XXX VALUE, --single CONFIG VALUE
                          Analyze single change: CONFIG_NAME y/m/n
    -d DIFF, --diff DIFF  Analyze changes from diff file

  example: ./tools/checkkconfig.py -f defconfig -s ELF n

  outputs:
  Change report for ELF=n
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
