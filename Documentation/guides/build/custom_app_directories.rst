===========================================
Custom Application Directories
===========================================

.. warning:: 
    Migrated from: 
    https://cwiki.apache.org/confluence/display/NUTTX/Custom+Application+Directories

Most people use the generic ``apps/`` directory with NuttX. That is convenient
and well-documented. However, it should always be remembered that NuttX is a
stand-alone, general-purpose OS and has **no dependency** on that "canned"
application directory.

This page shows how to create your own, custom application directory from
scratch.

Creating the Custom Application Directory
=========================================

Below is a simple example of the **minimum** custom application directory. It
contains only three files: ``Makefile``, ``Kconfig``, and ``hello.c``.

Makefile
--------

The custom application directory must include a ``Makefile`` that supports all
of the make targets expected by the NuttX build system **and** must generate an
archive called ``libapps.a`` in the top-level of the custom directory structure.
The minimal required targets for the ``Makefile`` look like this:

.. code-block:: shell

    APPDIR = ${shell pwd}
    
    -include $(TOPDIR)/Make.defs
    
    # files
    
    CSRCS = hello.c
    COBJS = hello.o
    
    ROOTDEPPATH = --dep-path .
    
    # Build targets
    
    all: libapps.a
    .PHONY: dirlinks context preconfig depend clean clean_context distclean
    .PRECIOUS: libapps$(LIBEXT)
    
    # Compile C Files
    
    $(COBJS): %$(OBJEXT): %.c
    $(call COMPILE, $<, $@)
    
    # Add object files to the apps archive
    
    libapps.a: $(COBJS)
    $(call ARCHIVE, libapps.a, $(COBJS))
    
    # Create directory links
    
    dirlinks:
    
    # Setup any special pre-build context
    
    context:
    
    # Setup any special pre-configuration context
    
    preconfig:
    
    # Make the dependency file, Make.deps
    
    depend: Makefile $(CSRCS)
    $(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
    
    # Clean the results of the last build
    
    clean:
    $(call CLEAN)
    
    # Remove the build context and directory links
    
    clean_context:
    
    # Restore the directory to its original state
    
    distclean: clean clean_context
    $(call DELFILE, Make.dep)
    
    # Include dependencies

    -include Make.dep
    

Kconfig
-------

A ``Kconfig`` file must be included, but it need not contain any meaningful
configuration options. This file is where you can add application-specific
configuration settings if desired. The minimal ``Kconfig`` might look like:

.. code-block:: shell

   # For a description of the syntax of this configuration file,
   # see the file kconfig-language.txt in the NuttX tools repository.
   #

hello.c
-------

Your custom application must compile at least one source file to generate the
required ``libapps.a`` archive. One of these source files must include the
``main()`` entry point to the application. That main function (or similarly
named entry point) is called after OS initialization completes.

What this application initialization entry point does, how it interacts with 
the rest of your application, and where the rest of you application code is 
located is of no concern to the OS. Only this one entry point is needed.

Below is a small "Hello, World!" example, where ``custom_main()`` is the
application entry point:

.. code-block:: c

    #include <stdio.h>
    
    int custom_main(int argc, char *argv[])
    {
    printf("Hello, World!!\n");
    return 0;
    }

Building with the Custom Application Directory
==============================================

In order to build with the new custom application directory, you need the
following in your NuttX configuration:

.. code-block:: shell

   CONFIG_APPS_DIR="../custom-apps"
   CONFIG_USER_ENTRYPOINT="custom_main"

.. note::

   You can only access the ``../custom-apps/Kconfig`` file if 
   ``CONFIG_APPS_DIR`` is set to ``../custom-apps`` **before** running
   ``make menuconfig``. If you start with an existing configuration, you may 
   face a "chicken-and-egg" situation. One workaround is to manually edit
   the ``.config`` file before running ``make menuconfig``.

Alternatively, if you use the ``tools/configure.sh`` script, you can specify the
custom-apps directory from the command line:

.. code-block:: shell

   tools/configure.sh -a ../custom_apps <board>:<config>

Afterward, just build NuttX as you normally would. When you run the program that
was built with your custom application directory, you should see:

.. code-block:: shell

   Hello, World!!
