=========
``nxpkg``
=========

``nxpkg`` manages standalone Dynamic ELF application images in a small,
versioned on-device store.  It can synchronize a static package catalog,
install or update a compatible payload, list available and installed
packages, roll back to the previous installed version, and remove a package.

The implementation is intentionally small.  It does not resolve dependencies
or provide a general-purpose system package format.

Configuration
=============

Enable ``CONFIG_SYSTEM_NXPKG``.  The command name, task settings, and storage
root are configured with:

* ``CONFIG_SYSTEM_NXPKG_PROGNAME``
* ``CONFIG_SYSTEM_NXPKG_PRIORITY``
* ``CONFIG_SYSTEM_NXPKG_STACKSIZE``
* ``CONFIG_SYSTEM_NXPKG_ROOT``

The default storage root is ``/var/lib/nxpkg``.  Mount persistent storage at
``/var`` or select a board-appropriate persistent path, such as
``/mnt/sdcard/nxpkg``, if installed packages must survive a reset.  A
``tmpfs``-backed root is useful for tests but is not persistent.

Dynamic ELF loading and the target's required binary-format support must also
be enabled.  Synchronizing or downloading from an HTTP URL requires
``CONFIG_NETUTILS_WEBCLIENT`` and a working network configuration.  Local
filesystem sources work without the web client.

Commands
========

The command-line interface is:

.. code-block:: console

   nxpkg sync <index-source>
   nxpkg available
   nxpkg install <name>
   nxpkg update <name>
   nxpkg list
   nxpkg rollback <name>
   nxpkg remove <name>

``sync`` accepts either a local index path or an HTTP/HTTPS URL.  ``install``
selects the newest catalog entry whose architecture and compatibility strings
match the running target.  ``update`` uses the same operation and therefore
installs the newest matching version.  ``rollback`` swaps the current and
previous installed versions.

Repository format
=================

A repository is a directory of static files.  No package-specific server is
required.  Its index contains a ``packages`` array, for example:

.. code-block:: json

   {
     "packages": [
       {
         "name": "hello",
         "version": "1.0.0",
         "arch": "xtensa",
         "compat": "esp32s3-xiao",
         "artifact": "artifacts/xtensa/esp32s3/esp32s3-xiao/hello/1.0.0/hello",
         "sha256": "<64 hexadecimal characters>",
         "type": "elf"
       }
     ]
   }

``name``, ``version``, ``arch``, ``compat``, ``artifact``, ``sha256``, and
``type`` are required.  Supported types are ``elf`` and ``shared-lib``.
``description``, ``category``, ``icon``, and ``launch_args`` are optional
metadata used by front ends.  Relative artifact and icon paths are resolved
from the synchronized index location.

The catalog may contain entries for several targets and versions.  Version
selection compares dot-separated numeric components numerically and other
components lexically; it is not a complete Semantic Versioning
implementation.

The repository export helper in ``apps/tools/export_pkg_repo.py`` copies
built artifacts, calculates their SHA-256 digests, and writes the index:

.. code-block:: console

   python3 apps/tools/export_pkg_repo.py /tmp/nxpkg-repo \
     --arch xtensa \
     --chip esp32s3 \
     --compat esp32s3-xiao \
     --package hello:1.0.0:elf:apps/bin/hello

The resulting directory can be served during local development with any
static file server, for example:

.. code-block:: console

   cd /tmp/nxpkg-repo
   python3 -m http.server 8000

Then synchronize the board from the development host:

.. code-block:: console

   nsh> nxpkg sync http://192.0.2.1:8000/index.json
   nsh> nxpkg available
   nsh> nxpkg install hello
   nsh> nxpkg list

Replace the example address with one reachable from the target.

On-device layout
================

Under ``CONFIG_SYSTEM_NXPKG_ROOT``, ``nxpkg`` stores:

* ``index.jsn``: the last synchronized catalog and the source used to resolve
  relative artifact and icon paths
* ``instpkg.jsn``: the authoritative installed-package database
* ``pkgs/<name>/<version>/``: versioned payload and manifest files
* ``pkgs/<name>/current`` and ``previous``: convenience mirrors of the
  database state
* ``tmp/``: bounded temporary downloads and atomic-write files

Install and update operations use per-package locks.  A separate lock
serializes changes to the shared installed database.  An interrupted
operation leaves transaction state that the next operation can clean up.

Security scope
==============

The SHA-256 field detects accidental corruption only when the index itself is
trusted.  If both the catalog and artifact arrive over unauthenticated HTTP,
an active attacker can replace both and provide a matching digest.  Use an
authenticated transport and a trusted endpoint for untrusted networks.

The current format does not include signed repository metadata or package
signatures.  Plain HTTP is therefore appropriate only for a trusted, isolated
development network where that risk is explicitly accepted.  Package
compatibility checks are target-selection checks, not a security boundary.

Current limitations
===================

``nxpkg`` does not currently provide dependency solving, repository metadata
signatures, package signatures, or a policy engine.  Filesystem constraints
also apply to the selected storage root; for example, a short-name-only FAT
configuration limits usable package and artifact names.
