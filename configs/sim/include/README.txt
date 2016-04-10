README
======

  Overview
  --------
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

  The encrypted passwords in the provided passwd file are only value if the
  TEA key is set to:  012345678 9abcdef0 012345678 9abcdef0.  Changes to either
  the key or the password word will require regeneration of the nsh_romfimg.h
  header file.

  Updating the ROMFS File System
  ------------------------------
  The content on the nsh_romfsimg.h header file is generated from a sample
  directory structure.  That directory structure is contained in the binary
  file etc.tar.gz and can be modified per the following steps:

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

  Replacing the Password File
  ---------------------------
  The sim/nsh configuration can also be used to create a new password file.
  First, make these configuration changes:

    1. Disable logins

       - CONFIG_NSH_CONSOLE_LOGIN=y
       + # CONFIG_NSH_CONSOLE_LOGIN is not set
         # CONFIG_NSH_TELNET_LOGIN is not set

    2. Move the password file to a write-able file system:

       - CONFIG_FSUTILS_PASSWD_PATH="/etc/passwd"
       + CONFIG_FSUTILS_PASSWD_PATH="/tmp/passwd"

    3. make the password file modifiable

       - CONFIG_FSUTILS_PASSWD_READONLY=y
       # CONFIG_FSUTILS_PASSWD_READONLY is not set

  Now rebuild the simulation.  No login should be required to enter the
  shell and you should find the 'useradd', 'userdel', and 'passwd' commands
  available in the help summary, provided that they are enabled.  Make
  certain that the 'useradd' command is not disabled:

      # CONFIG_NSH_DISABLE_USERADD is not set

  Use the NSH 'useradd' command to add new uses with new user passwords like:

    nsh> useradd <username> <password>

  Do this as many times as you would like.  Each time that you do this a new
  entry with an encrypted password will be added to the passwd file at
  /tmp/passwd.  You can see the passwd file like:

    nsh> cat /tmp/passwd

  When you are finished, you can simply copy the /tmp/passwd content from the
  'cat' command and paste it into an editor.  Make sure to remove any
  carriage returns that may have ended up on the file if you are using
  Windows.

  Then recreate the nsh_romfsimg.h file as described above.  In step 2,
  simply replace the old /etc/passwd file with the one in your editor.
  When you are finished, the new passwd file will be in the ROMFS file
  system at the path /etc/passwd.  When you restore the original NSH
  sim configuration, these are the passwords that will be used.
