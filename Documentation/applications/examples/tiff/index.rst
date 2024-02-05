=====================================
``tiff`` TIFF file generation example
=====================================

This is a simple unit test for the TIFF creation library at ``apps/graphic/tiff``.
It is configured to work in the Linux user-mode simulation and has not been
tested in any other environment.

At a minimum, to run in an embedded environment, you will probably have to
change the configured paths to the TIFF files defined in the example.

- ``CONFIG_EXAMPLES_TIFF_OUTFILE`` – Name of the resulting TIFF file. Default is
  ``/tmp/result.tif``.
- ``CONFIG_EXAMPLES_TIFF_TMPFILE1/2`` – Names of two temporaries files that will
  be used in the file creation. Defaults are ``/tmp/tmpfile1.dat`` and
  ``/tmp/tmpfile2.dat``.

The following must also be defined in your ``apps/`` configuration file: ::

  CONFIG_EXAMPLES_TIFF=y
  CONFIG_GRAPHICS_TIFF=y
