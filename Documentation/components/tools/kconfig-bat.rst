===============
``kconfig.bat``
===============

Recent versions of NuttX support building NuttX from a native Windows
CMD.exe shell.  But kconfig-frontends is a Linux tool and is not yet
available in the pure CMD.exe environment.  At this point, there are
only a few options for the Windows user (see the top-level README.txt
file).

You can, with some effort, run the Cygwin kconfig-mconf tool directly
in the CMD.exe shell.  In this case, you do not have to modify the
.config file, but there are other complexities:  You need to
temporarily set the Cygwin directories in the PATH variable and
then run kconfig-mconf outside of the Make system.

kconfig.bat is a Windows batch file at tools/kconfig.bat that automates
these steps.  It is used from the top-level NuttX directory like::

    tools/kconfig menuconfig

NOTE: There is currently an issue with accessing DOS environment
variables from the Cygwin kconfig-mconf running in the CMD.exe shell.
The following change to the top-level Kconfig file seems to work around
these problems::

     config APPSDIR
          string
     -   option env="APPSDIR"
     +   default "../apps"
