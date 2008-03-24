README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

4.11.0
  This is the only version that this code has been built against.
  
  Although it compiles without error, the 4.11.0 compiler generates
  bad code on one of the files, mm/mm_initialize.c.  Below is a simple work-
  around.

    --- mm/mm_initialize.c.SAVE	2008-02-13 08:06:46.833857700 -0600
    +++ mm/mm_initialize.c	2008-02-13 08:07:26.367608900 -0600
    @@ -94,8 +94,11 @@
    {
       int i;
 
    +#if 0 /* DO NOT CHECK IN */
       CHECK_ALLOCNODE_SIZE;
       CHECK_FREENODE_SIZE;
    +#endif
 
   /* Set up global variables */

Other Versions
  If you use any version of ZDS-II other than 4.11.0 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/ez80f910200kitg/*/setenv.sh and (2)
  configs/ez80f910200kitg/*/Make.defs.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- src/ and include/
    These directories contain common logic for all ez80f910200kitg
    configurations.

Variations on the basic ez80f910200kitg configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh ez80f910200kitg/<sub-directory>
   cd <nuttx-top-directgory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

- ostest
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

Check out any README.txt files in these <sub-directory>s.
