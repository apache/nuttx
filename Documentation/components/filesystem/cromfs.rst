======
cromfs
======

Overview
========

This directory contains the the CROMFS file system.  This is an in-memory
(meaning no block driver), read-only (meaning that can lie in FLASH) file
system.  It uses LZF decompression on data only (meta data is not
compressed).

It accesses the in-memory file system via directory memory reads and, hence,
can only reside in random access NOR-like FLASH.  It is intended for use
with on-chip FLASH available on most MCUs (the design could probably be
extended to access non-random-access FLASH as well, but those extensions
are not yet in place).

I do not have a good way to measure how much compression we get using LZF.
I have seen 37% compression reported in other applications, so I have to
accept that for now.  That means, for example, that you could have a file
system with 512Kb of data in only 322Kb of FLASH, giving you 190Kb to do
other things with.

LZF compression is not known for its high compression ratios, but rather
for fast decompression.  According to the author of the LZF decompression
routine, it is nearly as fast as a memcpy!

There is also a new tool at /tools/gencromfs.c that will generate binary
images for the NuttX CROMFS file system and and an example CROMFS file
system image at apps/examples/cromfs.  That example includes a test file
system that looks like::

  $ ls -Rl ../apps/examples/cromfs/cromfs
  ../apps/examples/cromfs/cromfs:
  total 2
  -rwxr--r--+ 1 spuda spuda 171 Mar 20 08:02 BaaBaaBlackSheep.txt
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:11 emptydir
  -rwxr--r--+ 1 spuda spuda 118 Mar 20 08:05 JackSprat.txt
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:06 testdir1
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:10 testdir2
  drwxrwxr-x+ 1 spuda spuda   0 Mar 20 08:05 testdir3
  ../apps/examples/cromfs/cromfs/emptydir:
  total 0
  ../apps/examples/cromfs/cromfs/testdir1:
  total 2
  -rwxr--r--+ 1 spuda spuda 249 Mar 20 08:03 DingDongDell.txt
  -rwxr--r--+ 1 spuda spuda 247 Mar 20 08:06 SeeSawMargorieDaw.txt
  ../apps/examples/cromfs/cromfs/testdir2:
  total 5
  -rwxr--r--+ 1 spuda spuda  118 Mar 20 08:04 HickoryDickoryDock.txt
  -rwxr--r--+ 1 spuda spuda 2082 Mar 20 08:10 TheThreeLittlePigs.txt
  ../apps/examples/cromfs/cromfs/testdir3:
  total 1
  -rwxr--r--+ 1 spuda spuda 138 Mar 20 08:05 JackBeNimble.txt

When built into NuttX and deployed on a target, it looks like::

  NuttShell (NSH) NuttX-7.24
  nsh> mount -t cromfs /mnt/cromfs
  nsh> ls -Rl /mnt/cromfs
  /mnt/cromfs:
   dr-xr-xr-x       0 .
   -rwxr--r--     171 BaaBaaBlackSheep.txt
   dr-xr-xr-x       0 emptydir/
   -rwxr--r--     118 JackSprat.txt
   dr-xr-xr-x       0 testdir1/
   dr-xr-xr-x       0 testdir2/
   dr-xr-xr-x       0 testdir3/
  /mnt/cromfs/emptydir:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
  /mnt/cromfs/testdir1:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     249 DingDongDell.txt
   -rwxr--r--     247 SeeSawMargorieDaw.txt
  /mnt/cromfs/testdir2:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     118 HickoryDickoryDock.txt
   -rwxr--r--    2082 TheThreeLittlePigs.txt
  /mnt/cromfs/testdir3:
   drwxrwxr-x       0 .
   dr-xr-xr-x       0 ..
   -rwxr--r--     138 JackBeNimble.txt
  nsh>

Everything I have tried works:  examining directories, catting files, etc.
The "." and ".." hard links also work::

  nsh> cd /mnt/cromfs
  nsh> cat emptydir/../testdir1/DingDongDell.txt
  Ding, dong, bell,
  Pussy's in the well.
  Who put her in?
  Little Johnny Green.

  Who pulled her out?
  Little Tommy Stout.
  What a naughty boy was that,
  To try to drown poor pussy cat,
  Who never did him any harm,
  And killed the mice in his father's barn.

  nsh>

gencromfs
=========

The genromfs program can be found in tools/.  It is a single C file called
gencromfs.c.  It can be built in this way::

    cd tools
    make -f Makefile.host gencromfs

The genromfs tool used to generate CROMFS file system images.  Usage is
simple::

    gencromfs <dir-path> <out-file>

Where::

    <dir-path> is the path to the directory will be at the root of the
      new CROMFS file system image.
    <out-file> the name of the generated, output C file.  This file must
      be compiled in order to generate the binary CROMFS file system
      image.

All of these steps are automated in the apps/examples/cromfs/Makefile.
Refer to that Makefile as an reference.

Architecture
============

The CROMFS file system is represented by an in-memory data structure.  This
structure is a "tree."  At the root of the tree is a "volume node" that
describes the overall operating system.  Other entities within the file
system are presented by other types of nodes:  hard links, directories, and
files.  These nodes are all described in fs/cromfs/cromfs.h.

In addition to general volume information, the volume node provides an
offset to the the "root directory".  The root directory, like all other
CROMFS directories is simply a singly linked list of other nodes:  hard link
nodes, directory nodes, and files.  This list is managed by "peer offsets":
Each node in the directory contains an offset to its peer in the same
directory.  This directory list is terminated with a zero offset.

The volume header lies at offset zero.  Hence, any offset to a node or data
block can be converted to an absolute address in the in-memory CROMFS image
by simply adding that offset to the well-known address of the volume header.

Each hard link, directory, and file node in the directory list includes
such a "peer offset" to the next node in the list.  Each node is followed
by the NUL-terminated name of the node.  Each node also holds an additional
offset.  Directory nodes contain a "child offset".  That is, the offset to
the first entry in another singly linked list of nodes comprising the sub-
directory.

Hard link nodes hold the "link offset" to the node which is the target of
the link.  The link offset may be an offset to another hard link node, to a
directory, or to a file node.  The directory link offset would refer the
first node in singly linked directory list that represents the directory.

File nodes provide file data.  The file name string is followed by a
variable length list of compressed data blocks.  In this case each
compressed data block begins with an LZF header as described in
include/lzf.h.

So, given this description, we could illustrate the sample CROMFS file
system above with these nodes (where V=volume node, H=Hard link node,
D=directory node, F=file node, D=Data block)::

  V
  `- +- H: .
     |
     +- F: BaaBaaBlackSheep.txt
     |  `- D,D,D,...D
     +- D: emptydir
     |  |- H: .
     |  `- H: ..
     +- F: JackSprat.txt
     |  `- D,D,D,...D
     +- D: testdir1
     |  |- H: .
     |  |- H: ..
     |  |- F: DingDongDell.txt
     |  |  `- D,D,D,...D
     |  `- F: SeeSawMargorieDaw.txt
     |     `- D,D,D,...D
     +- D: testdir2
     |  |- H: .
     |  |- H: ..
     |  |- F: HickoryDickoryDock.txt
     |  |  `- D,D,D,...D
     |  `- F: TheThreeLittlePigs.txt
     |     `- D,D,D,...D
     +- D: testdir3
        |- H: .
        |- H: ..
        `- F: JackBeNimble.txt
           `- D,D,D,...D

Where, for example::

  H: ..

    Represents a hard-link node with name ".."

  |
  +- D: testdir1
  |  |- H: .

    Represents a directory node named "testdir1".  The first node of the
    directory list is a hard link with name "."

  |
  +- F: JackSprat.txt
  |  `- D,D,D,...D

    Represents f file node named "JackSprat.txt" and is followed by some
    sequence of compressed data blocks, D.

Configuration
=============

To build the CROMFS file system, you would add the following to your
configuration:

1. Enable LZF (The other LZF settings apply only to compression
   and, hence, have no impact on CROMFS which only decompresses)::

     CONFIG_LIBC_LZF=y

   NOTE: This should be selected automatically when CONFIG_FS_CROMFS
   is enabled.

2. Enable the CROMFS file system::

     CONFIG_FS_CROMFS=y

3. Enable the apps/examples/cromfs example::

     CONFIG_EXAMPLES_CROMFS=y

   Or the apps/examples/elf example if you like::

     CONFIG_ELF=y
     # CONFIG_BINFMT_DISABLE is not set
     CONFIG_EXAMPLES_ELF=y
     CONFIG_EXAMPLES_ELF_CROMFS=y

   Or implement your own custom CROMFS file system that example as a
   guideline.
