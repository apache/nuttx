==================
Custom Apps How-to
==================

NuttX comes with a large number of Apps but, most likely, you will want to add your own.

There are various different options for this depending on your requirements.

  #. Replace the apps/ directory completely
  #. Extend the apps/ directory to include a new custom directory
  #. Include an additional custom directory outside of the main source trees

The following sections explain these 3 methods using a ``CustomHello.c`` app and
the directory ``CustomApps`` as an example.

.. Tip::
  If you make errors while setting this up and the build fails, it is most likely you'll
  need to run ``make clean`` and possibly even ``make distclean`` before rebuilding to
  ensure it works correctly.

1. Replace The Apps/ Directory Completely
=========================================

The CustomApps directory need only to contain the minimum three files:

  * ``Makefile``
  * ``Kconfig``
  * ``CustomHello.c``


1.1 Makefile
------------

The custom application directory must include a Makefile to make all of the
targets expected by the NuttX build and must generate an archive called
libapps.a in the top-level of the custom directory structure.

The Makefile has just those minimum required targets:

    .. code-block:: console

      APPDIR = ${shell pwd}

      -include $(TOPDIR)/Make.defs

      # files

      CSRCS = CustomHello.c
      COBJS = CustomHello.o

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
        $(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep

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

1.2 Kconfig
-----------

A Kconfig file must be included but need not be populated with any meaningful
options.
This is a place where you can add settings to generate customized builds of
your custom application and/or choose which of your apps to include.

In the minimum case, Kconfig is only:

    .. code-block:: console

      # For a description of the syntax of this configuration file,
      # see the file kconfig-language.txt in the NuttX tools repository.
      #

but it is more usual to include at least the basic information any NuttX app
requires, as well as anything else your app may need:

    .. code-block:: console

      # For a description of the syntax of this configuration file,
      # see the file kconfig-language.txt in the NuttX tools repository.
      #

      config CUSTOM_APPS_MY_APP
	        tristate "My App"
	        default n
	        ---help---
		      Enable My App
		
      if CUSTOM_APPS_MY_APP

      config CUSTOM_APPS_MY_APP_PROGNAME
      	  string "Program name"
      	  default "myapp"
      	  ---help---
      		    This is the name of the program that will be used when the NSH ELF
      		    program is installed.
      	
      config CUSTOM_APPS_MY_APP_PRIORITY
      	  int "My App task priority"
      	  default 100

      config CUSTOM_APPS_MY_APP_STACKSIZE
      	  int "My App stack size"
      	  default DEFAULT_TASK_STACKSIZE

      endif

1.3 CustomHello.c
-----------------

The custom application must actually compile some source files in order to generate the required
libapps.a archive. One of these source files must include the ``main()`` entry point to the
application.

The function of this main() entry point simply to bring-up the full application. It is called
at the completion of OS initialization.

What this application initialization entry point does, how it interacts with the rest of your
application, and where the rest of you application code is located is of no concern to the OS.

Only this one entry point is needed.

For this "Hello, Custom World!" application ``custom_hello()`` is the application entry point:

    .. code-block:: console

      #include <stdio.h>

      int custom_hello(int argc, char *argv[])
      {
        printf("Hello, Custom World!!\n");
        return 0;
      }

1.4 Building with the CustomApps Directory
------------------------------------------

In order to build with the new custom configuration, you will need the following in your configuration:

:menuselection:`CONFIG_APPS_DIR="../CustomApps"`

:menuselection:`CONFIG_INIT_ENTRYPOINT="custom_hello"`

Note that you can only access the ``../CustomApps/Kconfig`` configuration file if ``CONFIG_APPS_DIR`` is set
to ``../CustomApps`` BEFORE ``make menuconfig`` is executed

This can be done by

* hand-editing the .config file before running make menuconfig, which is rarely a good idea
* Using ``kconfig-tweak --set-str CONFIG_APPS_DIR ../CustomApps``
* select the CustomApps directory as a command line option at the time the board is configured:

      .. code-block:: console

        ./tools/configure.sh -a ../CustomApps <board>:<config>

  or

      .. code-block:: console

        .tools/configure.sh -l ../CustomBoards/MyCustomBoardName/MyCustomConfig

Then build as you normally would. When you execute the custom_hello app you should see:

  .. code-block:: console

    Hello, Custom World!!

2. Extend the apps/ directory to include a new custom directory
===============================================================

The collection of apps provided in nuttx-apps can be useful, and this method simply
extends the directory structure to include your own directory structure.

The existing /apps makefile automatically checks for the existence of sub-directories
that contain a ``Makefile`` and ``Make.defs`` file. This example assumes there is likely
to be more than one custom app, and includes a ``Kconfig`` for the app itself. Inclusion
of a ``Kconfig`` allows custom App options to be included in the NuttX configuration
system, but is optional.

2.1 Custom Apps Directory
-------------------------

Simply create a new directory under the existing apps directory with a name of your choice.
This example uses the directory name ``CustomApps``.

2.2 Make.defs
-------------

Create this file in the ``CustomApps`` directory, with the following line added:

  .. code-block:: console

    include $(wildcard $(APPDIR)/CustomApps/*/Make.defs)

2.3 Makefile
------------

Create a Makefile in the ``CustomApps`` directory, with the following lines added:

  .. code-block:: console

    MENUDESC = "Custom Apps"

    include $(APPDIR)/Directory.mk

2.4 CustomHello App
-------------------

Create a sub-directory under the ``CustomApps`` directory called ``CustomHello``.

The same ``CustomHello.c`` file as described above should be created here.

2.5 CustomHello Make.defs
-------------------------

Create a Make.defs in the ``CustomApps/CustomHello`` directory with the following lines:

  .. code-block:: console

    ifneq ($(CONFIG_CUSTOM_APPS_CUSTOM_HELLO),)
    CONFIGURED_APPS += $(APPDIR)/CustomApps/CustomHello
    endif


2.6 CustomHello Makefile
------------------------

Create a Makefile in the ``CustomApps/CustomHello`` directory with the following lines:

  .. code-block:: console

    include $(APPDIR)/Make.defs

    # Custom Hello built-in application info

    PROGNAME = $(CONFIG_CUSTOM_APPS_CUSTOM_HELLO_PROGNAME)
    PRIORITY = $(CONFIG_CUSTOM_APPS_CUSTOM_HELLO_PRIORITY)
    STACKSIZE = $(CONFIG_CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE)
    MODULE = $(CONFIG_CUSTOM_APPS_CUSTOM_HELLO)

    # Custom Hello

    MAINSRC = CustomHello.c

    include $(APPDIR)/Application.mk


2.7 CustomHello Kconfig
-----------------------

Create a Kconfig file in the ``CustomApps/CustomHello`` directory, with the following lines. For
the purposes of this example, the Kconfig will only cover our single application):

  .. code-block:: console

    #
    # For a description of the syntax of this configuration file,
    # see the file kconfig-language.txt in the NuttX tools repository.
    #

    config CUSTOM_APPS_CUSTOM_HELLO
	    tristate "Custom Hello App"
	    default n
	    ---help---
		    Enable the Custom Hello App

    if CUSTOM_APPS_CUSTOM_HELLO

    config CUSTOM_APPS_CUSTOM_HELLO_PROGNAME
	    string "Program name"
	    default "custom_hello"
	    ---help---
		    This is the name of the program that will be used when the NSH ELF
		    program is installed.

    config CUSTOM_APPS_CUSTOM_HELLO_PRIORITY
	    int "Custom Hello task priority"
	    default 100

    config CUSTOM_APPS_CUSTOM_HELLO_STACKSIZE
	    int "Custom Hello stack size"
	    default DEFAULT_TASK_STACKSIZE

    endif

2.8 Build and Run
-----------------

Once these files have been created, run a ``make clean`` (you may need to run ``make distclean``
followed by ``make menuconfig``. If successful there will be new Kconfig entries.

:menuselection:`Application Configuraration --> Custom Apps --> Custom Hello App`

Select the ``Custom Hello App`` and run the usual build process. If successful
you can run the newly included ``custom_hello`` app.

3. Include an Additional Custom directory Outside of the Main Source Trees
==========================================================================

Thia is similar to the previous approach, but places the ``CustomApps`` directory
outside of the default trees.

3.1 Create Custom Apps directory and a Symbolic Link
----------------------------------------------------

Create a directory for the custom apps in a location of your choosing. Then create A
symbolic link in the main nuttx/apps directory.

This example assumes this has been placed below the top NuttX folder, alongside the
default ``apps`` directory, i.e. ``nuttx/CustomApps``

  .. code-block:: console

    $ pwd
    /home/nuttx
    $ ls -1
    apps
    CustomBoards
    nuttx
    $ mkdir CustomApps
    $ ls -1
    apps
    CustomApps
    CustomBoards
    nuttx
    $ cd apps
    $ ln -s ../CustomApps CustomApps

3.2 Make.defs etc.
------------------

Follow all the steps as in sections 2.2 to 2.7 above, creating the exact same files but
placing then in the new ``CustomApps`` directory location created as described here.
