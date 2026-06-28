=========
``nxpkg``
=========

``nxpkg`` is a small local-first package helper for Dynamic ELF content on
NuttX. In its current form it focuses on a narrow executable-package flow:

- read package metadata from a local repository index
- verify package compatibility for the current runtime
- verify the staged artifact with SHA-256
- install the payload into a versioned on-device store
- record installed package state and expose it through ``list``

This first documentation page describes the current MVP only. Follow-up work
such as repository sync, dependency solving, shared-library packaging, and
rollback/update execution is intentionally out of scope for this initial
application-level slice.

Overview
========

``nxpkg`` is intended to work with NuttX systems that already support Dynamic
ELF loading. The current implementation assumes that:

- ELF payloads are produced separately from the base firmware image
- package metadata is available locally on the target
- the repository index can be written under ``/etc/nxpkg``
- the package store and cache live under a writable ``/var`` hierarchy

There is no separate ``nxpkg.conf`` file in this initial MVP. The only file
kept under ``/etc/nxpkg`` is the local repository index used by
``nxpkg install``.

The package lifecycle handled by the current MVP is intentionally simple:

1. load ``index.json`` from the local repository
2. select the latest matching package entry by name
3. verify runtime compatibility against the current arch/board identity
4. stage the package payload
5. verify the SHA-256 digest from the manifest
6. copy the payload into the versioned package store
7. update installed package metadata and ``current`` / ``previous`` pointers

Current Commands
================

The first ``nxpkg`` slice provides the following command surface:

- ``nxpkg install <name>``
- ``nxpkg list``
- ``nxpkg help``

The command parser also reserves ``update`` and ``rollback`` subcommands, but
they are not implemented in this initial unit.

On-Device Layout
================

The current implementation uses the following default paths:

- repository metadata: ``/etc/nxpkg/index.json``
- installed package database: ``/var/lib/nxpkg/installed.json``
- package payload store: ``/var/lib/nxpkg/pkgs``
- temporary staging area: ``/var/cache/nxpkg/pkg``

The temporary staging area is used for the artifact copy being verified and
installed during the current ``install`` flow. In this MVP it is not a general
package cache or unpack directory yet.

Each manifest entry currently describes:

- package name
- package version
- target architecture
- target board compatibility string
- payload artifact path
- SHA-256 digest
- payload type

The first validated payload type is executable ELF content. A single
``index.json`` may contain multiple entries for the same package name and
version as long as they target different runtime identities. ``nxpkg install``
filters those entries by the current ``arch`` and ``compat`` values first, and
then chooses the newest version among the matching entries.

Configuration
=============

Enable ``nxpkg`` in ``menuconfig`` with:

- ``CONFIG_SYSTEM_NXPKG``

The application also selects:

- ``CONFIG_NETUTILS_CJSON``

Useful runtime prerequisites for the current MVP are:

- Dynamic ELF support enabled in the target configuration
- ``CONFIG_FS_TMPFS`` if the validation target uses ``tmpfs``-backed ``/etc``
  and ``/var`` mounts for the first local repository flow
- a writable ``/etc`` mount for the local repository index
- a writable ``/var`` mount for the package store/cache

The command name, task priority, and stack size can be adjusted with:

- ``CONFIG_SYSTEM_NXPKG_PROGNAME``
- ``CONFIG_SYSTEM_NXPKG_PRIORITY``
- ``CONFIG_SYSTEM_NXPKG_STACKSIZE``

Basic Usage
===========

Running ``nxpkg`` without a subcommand prints the usage line::

  nsh> nxpkg
  Usage: nxpkg <install|update|list|rollback|help> [args]

To inspect installed package state::

  nsh> nxpkg list

The local install path expects a repository index at ``/etc/nxpkg/index.json``.
For the initial validation flow described below, the index is copied from the
ELF ROMFS fixture generated at build time.

Generating a Local Repository
=============================

The current ``nxpkg`` workflow expects a repository directory containing an
``index.json`` file plus the payload artifacts referenced by that index. A
host-side helper script is available in ``apps/tools/export_pkg_repo.py`` to
generate this layout from built artifacts.

For example, after building the ``hello`` ELF payload, the following command
exports a repository for the XIAO validation target::

  python3 apps/tools/export_pkg_repo.py /tmp/nxpkg-repo \
    --arch xtensa \
    --chip esp32s3 \
    --compat esp32s3-xiao \
    --package hello:1.0.0:elf:apps/bin/hello

This produces a repository layout like::

  /tmp/nxpkg-repo/
  ├── index.json
  └── artifacts/
      └── xtensa
          └── esp32s3
              └── esp32s3-xiao
                  └── hello
                      └── 1.0.0
                          └── hello

The generated ``index.json`` records the package metadata, payload type, and
SHA-256 digest expected by the current ``nxpkg install`` path. The artifact
subtree mirrors the ``nuttx/boards`` hierarchy by keeping the
``arch/chip/board`` levels ahead of the package path.

If the repository is going to be served by a static transport later, such as
FTP or HTTP, ``export_pkg_repo.py`` also supports ``--artifact-prefix`` so the
artifact paths written into ``index.json`` can be prefixed for that server
layout.

Validated Runtime Flow
======================

The current target-side validation flow was exercised on:

- board: Seeed XIAO ESP32S3 Sense
- configuration: ``esp32s3-xiao:elf``

The executable loader baseline was first validated with the
:doc:`../../examples/elf/index` example. After that, the same
ELF ROMFS image was used to provide a local package fixture for ``nxpkg``.

The ROMFS-backed validation script used on the board was::

  mount -t tmpfs /etc
  mount -t tmpfs /var
  mkdir /etc/nxpkg
  cp /mnt/elf/romfs/index.json /etc/nxpkg/index.json
  nxpkg install hello
  nxpkg list

The runtime session used for validation was:

1. boot to NSH over USB CDC
2. confirm that ``nxpkg`` is present and prints usage
3. run ``elf`` so the ROMFS fixture becomes available at ``/mnt/elf/romfs``
4. run the package validation script shown above
5. confirm that ``nxpkg install hello`` completes successfully
6. confirm that ``nxpkg list`` reports the installed package state

The observed target-side result was::

  nsh> nxpkg
  Usage: nxpkg <install|update|list|rollback|help> [args]

  nsh> elf
  ... ELF test payloads completed and returned to nsh>

  nsh> source /mnt/elf/romfs/pkgtest.nsh
  nxpkg: info: layout prepared
  nxpkg: info: loading index from /etc/nxpkg/index.json
  nxpkg: info: index read complete (213 bytes)
  nxpkg: info: cJSON_Parse returned success
  nxpkg: info: parsed manifest hello 1.0.0
  nxpkg: info: selected hello version 1.0.0
  nxpkg: info: artifact source /mnt/elf/romfs/hello
  nxpkg: info: artifact copied to staging
  nxpkg: info: sha256 verified
  nxpkg: info: payload staged at /var/lib/nxpkg/pkgs/hello/1.0.0/hello
  nxpkg: info: manifest written
  nxpkg: info: compatibility check passed
  nxpkg: info: installed metadata updated
  nxpkg: info: installed hello version 1.0.0
  hello current=1.0.0 previous=- type=elf arch=xtensa compat=esp32s3-xiao versions=1.0.0

  nsh> nxpkg list
  hello current=1.0.0 previous=- type=elf arch=xtensa compat=esp32s3-xiao versions=1.0.0

During this validation, the XIAO ``esp32s3-xiao:elf`` configuration needed
explicit writable mounts for both ``/etc`` and ``/var``. The fixture mounted
``tmpfs`` on those paths first, created ``/etc/nxpkg``, and then copied the
local ``index.json`` into place before running ``nxpkg install``.

Current Limitations
===================

The initial ``nxpkg`` MVP deliberately does not yet provide:

- remote repository sync or download support
- dependency solving
- shared-library package installation
- ``update`` execution
- ``rollback`` execution
- final integration with the NuttX cryptographic subsystem

Those items are expected to follow in later work once the initial
application-level install/list flow is reviewed and stabilized.
