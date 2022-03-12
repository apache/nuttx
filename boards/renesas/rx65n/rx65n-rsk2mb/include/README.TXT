README
======

  Overview
  --------
  This directory contains logic to support a custom ROMFS start-up script.
  This startup script is used by by the NSH when it starts provided that
  CONFIG_NSH_ARCHROMFS=y.  The script provides a ROMFS volume that will be
  mounted at /etc and will look like this at run-time:

    NuttShell (NSH) NuttX-8.2
    nsh> ls -l /etc
    /etc:
     dr-xr-xr-x       0 .
     -r--r--r--      20 group
     dr-xr-xr-x       0 init.d/
     -r--r--r--      35 passwd
    /etc/init.d:
     dr-xr-xr-x       0 ..
     -r--r--r--     110 rcS
    nsh>

  /etc/init.d/rcS is the start-up script; /etc/passwd is a the password
  file.  It supports a single user:

    USERNAME:  admin
    PASSWORD:  Adminstrator

    nsh> cat /etc/passwd
    admin:8Tv+Hbmr3pLddSjtzL0kwC:0:0:/

  The encrypted passwords in the provided passwd file are only valid if the
  TEA key is set to:  012345678 9abcdef0 012345678 9abcdef0.  Changes to either
  the key or the password word will require regeneration of the nsh_romfimg.h
  header file.

  The format of the password file is:

    user:x:uid:gid:home

  Where:
    user:  User name
    x:     Encrypted password
    uid:   User ID (0 for now)
    gid:   Group ID (0 for now)
    home:  Login directory (/ for now)

  /etc/group is a group file.  It is not currently used.

    nsh> cat /etc/group
    root:*:0:root,admin

  The format of the group file is:

    group:x:gid:users

  Where:
    group:  The group name
    x:      Group password
    gid:    Group ID
    users:  A comma separated list of members of the group

  Updating the ROMFS File System
  ------------------------------
  The content on the nsh_romfsimg.h header file is generated from a sample
  directory structure.  That directory structure is contained in the etc/ directory and can be modified per the following steps:

    1. Change directory to etc/:

       cd etc/

    2. Make modifications as desired.

    3. Create the new ROMFS image.

       genromfs -f romfs_img -d etc -V SimEtcVol

    4. Convert the ROMFS image to a C header file

        xxd -i romfs_img >nsh_romfsimg.h

    5. Edit nsh_romfsimg.h, mark both data definitions as 'const' so that
       that will be stored in FLASH.

