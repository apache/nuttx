This directory provides a build area for all Renesas and legacy Hitachi
architectures.  The 'common' subdirectory contains source files shared by
all Renesas architectures; Source files unique to a specific Renesas chip
architecture are contained in a subdirectory named after the chip.  At
configuration time, additional directories will be linked here:  'board'
will be a link to the configs/*/src directory; 'chip' will be a link to
the SH chip sub-directory.
