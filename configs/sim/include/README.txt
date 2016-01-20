README
======

  This directory contains logic to support a custom ROMFS start-up script.
  This startup script is used by by the NSH when it starts provided that
  CONFIG_NSH_ARCHROMFS=y.  The script provides a ROMFS volue that will be
  mounted at /etc and will look like this at run-time:

    NuttShell (NSH) NuttX-7.13
    This is the message of the day
    nsh> ls -Rl /etc
    /etc:
     dr-xr-xr-x       0 .
     dr-xr-xr-x       0 init.d/
     -r--r--r--      39 passwd
    /etc/init.d:
     dr-xr-xr-x       0 ..
     -r--r--r--     110 rcS
    nsh>

  /etc/init.d/rcS is the start-up script; /etc/passwd is a the password
  file.  It supports a single user:

    USERNAME:  admin
    PASSWORD:  Adminstrator

  This directory structure is contained in the binary file etc.tar.gz and
  can be modified per the following steps:

    1. Unpack the directory structure:

       tar zxf etc.tar.gz

    2. Make modifications as desired.

    3. Create the new ROMFS image.

       genromfs -f romfs_img -d etc -V SimEtcVol

    4. Convert the ROMFS image to a C header file

        xxd -i romfs_img >nsh_romfsimg.h

    5. Edit nsh_romfsimg.h, mark both data definitions as 'const' so that
       that will be stored in FLASH.

    6. Re-package the modified /etc directory

       tar cvf etc.tar etc
       gzip etc.tar

  See the sim/nsh configuration for an example of the use of this file
  system.
