================================
Creating an Apache NuttX Release
================================

NuttX releases are targeted for once every 3 months.

Checkout the distribution SVN repositories
==========================================

Releases are managed through an SVN repository. There are two locations where
releases can be committed dev and release. Prior to voting a release is staged
in the dev folder after a release is approved by the IPMC it is then moved to
the release location and committed for distribution. The release folder also
holds the GPG public keys that are used for signing release in a KEYS file.

.. code-block:: console

  $ svn checkout https://dist.apache.org/repos/dist/dev/nuttx nuttx-dev
  $ svn checkout https://dist.apache.org/repos/dist/release/nuttx nuttx-release

Adding your GPG key
===================

Inside of the ``dist/release/nuttx folder`` is a KEYS file where committers must
upload their GPG public key that they use to sign releases. On the top of the
file you can see instructions on how to add your key to this file. Be careful to
not remove any existing keys. There is a KEYS file in both the dev and releases
folder, but uploading to the releases folder is the important one.

If you have not created a GPG key for use with this project see
https://infra.apache.org/openpgp.html#generate-key It is important that your
Apache email is associated with this key.

My key id is ``3554D78458CEB6954B020E12E1B6E30DB05D6280``. You can list the keys
that you have a secret key for with this command. Make sure your Apache email is
associated with this key.

.. code-block:: console

   $ gpg2 --list-secret-keys
    /home/bashton/.gnupg/pubring.kbx
    --------------------------------
    sec>  rsa4096 2019-11-24 [SC] [expires: 2021-09-02]
          3554D78458CEB6954B020E12E1B6E30DB05D6280
          Card serial no. = 0006 09239558
    uid           [ultimate] Brennan Ashton <btashton@apache.org>
    uid           [ultimate] Brennan Ashton <bashton@brennanashton.com>
    ssb>  rsa4096 2019-11-24 [E] [expires: 2021-09-02]
    ssb>  rsa4096 2019-11-24 [A] [expires: 2021-09-02]
    ssb   rsa4096 2019-11-24 [S] [expires: 2021-09-02]

You can then use this command to add to the KEYS file (fill in "key id" with your key id):

.. code-block:: console

   $ (gpg --list-sigs <key id> && gpg --armor --export <key id>) >> KEYS

You can verify your key is in the file with:

.. code-block:: console

   $ cat KEYS | gpg2 --import-options show-only

Once you are happy with your changes you can commit your key

.. code-block:: console

   $ svn commit -m "Update <my name> GPG key"

Add your GPG key to GitHub / Apache
===================================

So that the release tags show up as "verified" attach your GPG key to your
Apache and GitHub accounts:

* GitHub: https://docs.github.com/en/github/authenticating-to-github/adding-a-new-gpg-key-to-your-github-account

* Apache: https://id.apache.org

  * *Add the fingerprint to OpenPGP Public Key Primary Fingerprint*

Creating a Release Candidate
============================

When the project is happy with a release branch and is ready to create a release
candidate, the first step is to create a signed tag. This should be done for
both the nuttx and nuttx-apps repositories.

This is an example for tagging RC0 for the 12.1.0 release. Only the OS
repository is shown here this must also be done for the apps repository.

.. code-block:: console

   # Export the signing key
   $ export GPG_TTY=$(tty)

   # Checkout the release branch
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git checkout releases/12.1
   Already on 'releases/12.1'
   Your branch is up to date with 'origin/releases/12.1'.

   # Make sure it is up-to-date with upstream
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git pull
   Already up to date.

   # Make create the signed tag (note the -s option)
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git tag -s nuttx-12.1.0-RC0 -m nuttx-12.1.0-RC0

   # Verify the tag is on the correct commit
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git log -n 1
   commit 16748108c503d762779545d40113825e54b75252 (HEAD -> releases/12.1, tag: nuttx-12.1.0-RC0, origin/releases/12.1)
   Author: Dong Heng <dongheng@espressif.com>
   Date:   Fri Apr 9 20:03:24 2021 +0800

       riscv/esp32c3: Fix heap end address

   # Push the tag to the apache repository
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git push -u origin nuttx-12.1.0-RC0
   Enumerating objects: 1, done.
   Counting objects: 100% (1/1), done.
   Writing objects: 100% (1/1), 805 bytes | 402.00 KiB/s, done.
   Total 1 (delta 0), reused 0 (delta 0), pack-reused 0
   To github.com:apache/nuttx.git
    * [new tag]               nuttx-12.1.0-RC0 -> nuttx-12.1.0-RC0

You should be able to see the tag here https://github.com/apache/nuttx/tags and
https://github.com/apache/nuttx-apps/tags.

Creating the Release Tarballs
=============================

Make sure that you have both repositories checked to the correct release
candidate tag. The folder names must be ``nuttx`` and ``apps``.

.. code-block:: console

   ~/nuttx/wrk/release 
   $ ls
   apps  nuttx

   ~/nuttx/wrk/release 
   $ git -C nuttx log -n 1
   commit 16748108c503d762779545d40113825e54b75252 (HEAD -> releases/12.1, tag: nuttx-12.1.0-RC0, origin/releases/12.1)
   Author: Dong Heng <dongheng@espressif.com>
   Date:   Fri Apr 9 20:03:24 2021 +0800

       riscv/esp32c3: Fix heap end address

   ~/nuttx/wrk/release 
   $ git -C apps log -n 1
   commit 4348d91d1356335483089c3865282d80f13bedcd (HEAD -> releases/12.1, tag: nuttx-12.1.0-RC0, origin/releases/12.1)
   Author: Abdelatif Guettouche <abdelatif.guettouche@espressif.com>
   Date:   Mon Apr 12 10:11:05 2021 +0200

       wireless/wapi/src/wapi.c: When executing a command return it's error code on failure.
       
       Signed-off-by: Abdelatif Guettouche <abdelatif.guettouche@espressif.com>

When creating the release tarballs consider enabling debug mode with the ``-d``
flag to make sure everything looks correct including using the correct folders.
Note that here we do not use the RC in the version. If this RC is accepted these
exact files will be moved from dev to the release folder, the tarballs are *not*
recreated. Here is an example signing using my key id and the 12.1.0 release:

.. code-block:: console

   ~/nuttx/wrk/release took 2s 
   $ ./nuttx/tools/zipme.sh -d -s -k 3554D78458CEB6954B020E12E1B6E30DB05D6280 12.1.0
   + DEBUG=-d
   + shift
   + '[' '!' -z -s ']'
   + case $1 in
   + sign=1
   + shift
   + '[' '!' -z -k ']'
   + case $1 in
   + shift
   + GPG+=' --default-key 3554D78458CEB6954B020E12E1B6E30DB05D6280'
   + shift
   + '[' '!' -z 12.1.0 ']'
   + case $1 in
   + break
   + VERSION=12.1.0
   + '[' -n 12.1.0 ']'
   + VERSIONOPT='-v 12.1.0'
   + for pat in ${EXCLPAT}
   + TAR+=' --exclude=.github'
   + for pat in ${EXCLPAT}
   + TAR+=' --exclude=.asf.yaml'
   + TAR+=' --exclude-vcs'
   + '[' 0 '!=' 0 ']'
   + TAR+=' -czf'
   ++ basename ./nuttx/tools/zipme.sh
   + MYNAME=zipme.sh
   + '[' -x /home/bashton/nuttx/wrk/release/zipme.sh ']'
   + '[' -x /home/bashton/nuttx/wrk/release/tools/zipme.sh ']'
   + '[' -x /home/bashton/nuttx/wrk/release/nuttx/tools/zipme.sh ']'
   + TRUNKDIR=/home/bashton/nuttx/wrk/release
   + NUTTXDIR=/home/bashton/nuttx/wrk/release/nuttx
   + APPSDIR=/home/bashton/nuttx/wrk/release/apps
   + '[' '!' -d /home/bashton/nuttx/wrk/release ']'
   + cd /home/bashton/nuttx/wrk/release
   + '[' '!' -d /home/bashton/nuttx/wrk/release/nuttx ']'
   + '[' '!' -d /home/bashton/nuttx/wrk/release/apps ']'
   + echo 'Cleaning the repositories'
   Cleaning the repositories
   + '[' 0 '!=' 0 ']'
   + make -C /home/bashton/nuttx/wrk/release/nuttx distclean
   + VERSIONSH=/home/bashton/nuttx/wrk/release/nuttx/tools/version.sh
   + '[' '!' -x /home/bashton/nuttx/wrk/release/nuttx/tools/version.sh ']'
   + /home/bashton/nuttx/wrk/release/nuttx/tools/version.sh -d -v 12.1.0 /home/bashton/nuttx/wrk/release/nuttx/.version
   + shift
   + '[' '!' -z -v ']'
   + case $1 in
   + shift
   + VERSION=12.1.0
   + shift
   + '[' '!' -z /home/bashton/nuttx/wrk/release/nuttx/.version ']'
   + case $1 in
   + break
   + OUTFILE=/home/bashton/nuttx/wrk/release/nuttx/.version
   + '[' -z 12.1.0 ']'
   + '[' -z 12.1.0 ']'
   + '[' -z /home/bashton/nuttx/wrk/release/nuttx/.version ']'
   ++ echo 12.1.0
   ++ cut -d. -f1
   + MAJOR=10
   + '[' X10 = X12.1.0 ']'
   ++ echo 12.1.0
   ++ cut -d. -f2
   + MINOR=1
   + '[' X12.1 = X12.1.0 ']'
   ++ echo 12.1.0
   ++ grep -Eo '[0-9]+\.[0-9]+\.[0-9]+'
   ++ cut -d. -f3
   + PATCH=0
   + '[' -z '' ']'
   ++ git -C /home/bashton/nuttx/wrk/release/nuttx/tools log --oneline -1
   ++ cut '-d ' -f1
   + BUILD=16748108c5
   + '[' -z 16748108c5 ']'
   ++ git -C /home/bashton/nuttx/wrk/release/nuttx/tools diff-index --name-only HEAD
   ++ head -1
   + '[' -n '' ']'
   + echo '#!/bin/bash'
   + echo ''
   + echo 'CONFIG_VERSION_STRING="12.1.0"'
   + echo CONFIG_VERSION_MAJOR=10
   + echo CONFIG_VERSION_MINOR=1
   + echo CONFIG_VERSION_PATCH=0
   + echo 'CONFIG_VERSION_BUILD="16748108c5"'
   + chmod 755 /home/bashton/nuttx/wrk/release/nuttx/.version
   + '[' -z 12.1.0 ']'
   + NUTTX_TARNAME=apache-nuttx-12.1.0.tar
   + APPS_TARNAME=apache-nuttx-apps-12.1.0.tar
   + NUTTX_ZIPNAME=apache-nuttx-12.1.0.tar.gz
   + APPS_ZIPNAME=apache-nuttx-apps-12.1.0.tar.gz
   + NUTTX_ASCNAME=apache-nuttx-12.1.0.tar.gz.asc
   + APPS_ASCNAME=apache-nuttx-apps-12.1.0.tar.gz.asc
   + NUTTX_SHANAME=apache-nuttx-12.1.0.tar.gz.sha512
   + APPS_SHANAME=apache-nuttx-apps-12.1.0.tar.gz.sha512
   + '[' -f apache-nuttx-12.1.0.tar ']'
   + '[' -f apache-nuttx-12.1.0.tar.gz ']'
   + echo 'Removing /home/bashton/nuttx/wrk/release/apache-nuttx-12.1.0.tar.gz'
   Removing /home/bashton/nuttx/wrk/release/apache-nuttx-12.1.0.tar.gz
   + rm -f apache-nuttx-12.1.0.tar.gz
   + '[' -f apache-nuttx-apps-12.1.0.tar ']'
   + '[' -f apache-nuttx-apps-12.1.0.tar.gz ']'
   + '[' -f apache-nuttx-12.1.0.tar.gz.asc ']'
   + '[' -f apache-nuttx-apps-12.1.0.tar.gz.asc ']'
   + '[' -f apache-nuttx-12.1.0.tar.gz.sha512 ']'
   + '[' -f apache-nuttx-apps-12.1.0.tar.gz.sha512 ']'
   + echo 'Archiving and zipping nuttx/'
   Archiving and zipping nuttx/
   ++ basename /home/bashton/nuttx/wrk/release/nuttx
   + tar --exclude=.github --exclude=.asf.yaml --exclude-vcs -czf apache-nuttx-12.1.0.tar.gz nuttx
   + echo 'Archiving and zipping apps/'
   Archiving and zipping apps/
   ++ basename /home/bashton/nuttx/wrk/release/apps
   + tar --exclude=.github --exclude=.asf.yaml --exclude-vcs -czf apache-nuttx-apps-12.1.0.tar.gz apps
   + echo 'Creating the hashes'
   Creating the hashes
   + sha512sum apache-nuttx-12.1.0.tar.gz
   + sha512sum apache-nuttx-apps-12.1.0.tar.gz
   + '[' 1 '!=' 0 ']'
   + echo 'Signing the tarballs'
   Signing the tarballs
   + gpg -sab --default-key 3554D78458CEB6954B020E12E1B6E30DB05D6280 apache-nuttx-12.1.0.tar.gz
   gpg: using "3554D78458CEB6954B020E12E1B6E30DB05D6280" as default secret key for signing
   + gpg -sab --default-key 3554D78458CEB6954B020E12E1B6E30DB05D6280 apache-nuttx-apps-12.1.0.tar.gz
   gpg: using "3554D78458CEB6954B020E12E1B6E30DB05D6280" as default secret key for signing
   + cd /home/bashton/nuttx/wrk/release/nuttx

   ~/nuttx/wrk/release took 6s 
   $ ls
   apache-nuttx-12.1.0.tar.gz      apache-nuttx-12.1.0.tar.gz.sha512  apache-nuttx-apps-12.1.0.tar.gz.asc     apps
   apache-nuttx-12.1.0.tar.gz.asc  apache-nuttx-apps-12.1.0.tar.gz    apache-nuttx-apps-12.1.0.tar.gz.sha512  nuttx

Check the release artifacts
===========================

Prior to uploading the artifacts it is a good idea to make sure that they pass a
sanity check. You can do this by running the ``nuttx/tools/checkrelease.sh``
script on them. This will only use the GPG keys at
https://dist.apache.org/repos/dist/dev/nuttx/KEYS so make sure.

.. code-block:: console

   ~/nuttx/wrk/release 
   $ ./nuttx/tools/checkrelease.sh --dir ./
   gpg: directory '/tmp/nuttx-checkrelease/.gnupg' created
   gpg: keybox '/tmp/nuttx-checkrelease/.gnupg/pubring.kbx' created
   gpg: /tmp/nuttx-checkrelease/.gnupg/trustdb.gpg: trustdb created
   gpg: key E1B6E30DB05D6280: public key "Brennan Ashton <btashton@apache.org>" imported
   gpg: Total number processed: 1
   gpg:               imported: 1
    OK: https://dist.apache.org/repos/dist/dev/nuttx/KEYS is imported.
   Checking apache-nuttx-12.1.0.tar.gz sha512...
    OK: apache-nuttx-12.1.0.tar.gz sha512 hash matches.

   Checking apache-nuttx-12.1.0.tar.gz GPG signature:
   gpg: Signature made Sat 17 Apr 2021 08:02:29 PM PDT
   gpg:                using RSA key 66C4832A165ECC9354895A209750ED7E692B99E2
   gpg: Good signature from "Brennan Ashton <btashton@apache.org>" [unknown]
   gpg:                 aka "Brennan Ashton <bashton@brennanashton.com>" [unknown]
   gpg: WARNING: This key is not certified with a trusted signature!
   gpg:          There is no indication that the signature belongs to the owner.
   Primary key fingerprint: 3554 D784 58CE B695 4B02  0E12 E1B6 E30D B05D 6280
        Subkey fingerprint: 66C4 832A 165E CC93 5489  5A20 9750 ED7E 692B 99E2
    OK: apache-nuttx-12.1.0.tar.gz gpg signature matches.

   Checking apache-nuttx-12.1.0.tar.gz for required files:
    OK: all required files exist in nuttx.

   Checking apache-nuttx-apps-12.1.0.tar.gz sha512...
    OK: apache-nuttx-apps-12.1.0.tar.gz sha512 hash matches.

   Checking apache-nuttx-apps-12.1.0.tar.gz GPG signature:
   gpg: Signature made Sat 17 Apr 2021 08:02:30 PM PDT
   gpg:                using RSA key 66C4832A165ECC9354895A209750ED7E692B99E2
   gpg: Good signature from "Brennan Ashton <btashton@apache.org>" [unknown]
   gpg:                 aka "Brennan Ashton <bashton@brennanashton.com>" [unknown]
   gpg: WARNING: This key is not certified with a trusted signature!
   gpg:          There is no indication that the signature belongs to the owner.
   Primary key fingerprint: 3554 D784 58CE B695 4B02  0E12 E1B6 E30D B05D 6280
        Subkey fingerprint: 66C4 832A 165E CC93 5489  5A20 9750 ED7E 692B 99E2
    OK: apache-nuttx-apps-12.1.0.tar.gz gpg signature matches.

   Checking apache-nuttx-apps-12.1.0.tar.gz for required files:
    OK: all required files exist in apps.

   Trying to build nuttx sim:nsh...
    OK: we were able to build sim:nsh.

Staging the release candidate
=============================

To stage a release a new folder should be created under
https://dist.apache.org/repos/dist/dev/nuttx for the release candidate and these
release artifacts should be copied there:

.. code-block:: console

    apache-nuttx-<version>.tar.gz      apache-nuttx-<version>.tar.gz.sha512  apache-nuttx-apps-<version>.tar.gz.asc
    apache-nuttx-<version>.tar.gz.asc  apache-nuttx-apps-<version>.tar.gz    apache-nuttx-apps-<version>.tar.gz.sha512

If you checked that svn repository out as shown earlier as nuttx-dev. This
should be done like this:

.. code-block:: console

   ~/nuttx/svn/nuttx-dev 
   $ mkdir 12.1.0-RC0
   
   ~/nuttx/svn/nuttx-dev 
   $ cp -v ../../wrk/release/apache-{nuttx,nuttx-apps}-12.1.0.tar.gz* ./12.1.0-RC0/
   '../../wrk/release/apache-nuttx-12.1.0.tar.gz' -> './12.1.0-RC0/apache-nuttx-12.1.0.tar.gz'
   '../../wrk/release/apache-nuttx-12.1.0.tar.gz.asc' -> './12.1.0-RC0/apache-nuttx-12.1.0.tar.gz.asc'
   '../../wrk/release/apache-nuttx-12.1.0.tar.gz.sha512' -> './12.1.0-RC0/apache-nuttx-12.1.0.tar.gz.sha512'
   '../../wrk/release/apache-nuttx-apps-12.1.0.tar.gz' -> './12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz'
   '../../wrk/release/apache-nuttx-apps-12.1.0.tar.gz.asc' -> './12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz.asc'
   '../../wrk/release/apache-nuttx-apps-12.1.0.tar.gz.sha512' -> './12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz.sha512'

Then commit these files:

.. code-block:: console

   ~/nuttx/svn/nuttx-dev 
   $ svn status
   ?       12.1.0-RC0
   
   ~/nuttx/svn/nuttx-dev 
   $ svn add 12.1.0-RC0/
   A         12.1.0-RC0
   A         12.1.0-RC0/apache-nuttx-12.1.0.tar.gz.sha512
   A         12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz.sha512
   A  (bin)  12.1.0-RC0/apache-nuttx-12.1.0.tar.gz.asc
   A  (bin)  12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz.asc
   A  (bin)  12.1.0-RC0/apache-nuttx-12.1.0.tar.gz
   A  (bin)  12.1.0-RC0/apache-nuttx-apps-12.1.0.tar.gz
   
   ~/nuttx/svn/nuttx-dev 
   $ svn commit -m "Staging apache-nuttx-12.1.0-RC0"

Verify the release exists under https://dist.apache.org/repos/dist/dev/nuttx/

Call for a Community Vote
=========================

To do this send an email that looks something like this:

.. code-block:: text

   Subject: [VOTE] Apache NuttX 12.1.0 RC0 release
   To: dev@nuttx.apache.org

   Hello all,
   Apache NuttX 12.1.0 RC0 has been staged under [1] and it's
   time to vote on accepting it for release. If approved we will seek
   final release approval from the IPMC. Voting will be open for 72hr.

   A minimum of 3 binding +1 votes and more binding +1 than binding -1 are
   required to pass.

   The Apache requirements for approving a release can be found here [3]
   "Before voting +1 [P]PMC members are required to download the signed
   source code package, compile it as provided, and test the resulting
   executable on their own platform, along with also verifying that the
   package meets the requirements of the ASF policy on releases."

   A document to walk through some of this process has been published on
   our project wiki and can be found here [4].

   [ ] +1 accept (indicate what you validated - e.g. performed the non-RM
   items in [4])
   [ ] -1 reject (explanation required)

   Thank you all,
   <Release Manager>

   SCM Information:
     Release tag: nuttx-12.1.0-RC0
     Hash for the release nuttx tag: <GIT HASH>
     Hash for the release nuttx-apps tag: <GIT HASH>

   [1] https://dist.apache.org/repos/dist/dev/nuttx/12.1.0-RC0/
   [2] https://raw.githubusercontent.com/apache/nuttx/nuttx-12.1.0-RC0/ReleaseNotes
   [3] https://www.apache.org/dev/release.html#approving-a-release
   [4] https://cwiki.apache.org/confluence/display/NUTTX/Validating+a+staged+Release

After the voting requirements have been met (see the email text) the release an
email is sent closing out the voting.

Example text for that email is here. Note you will have to fill in the vote
count and an archive link to the voting thread. The best way to find the link is
here https://lists.apache.org/list.html?dev@nuttx.apache.org

.. code-block:: text

   Subject: [RESULTS][VOTE] Release Apache NuttX 12.1.0 [RC0]
   To: dev@nuttx.apache.org
   
   Hi all,
   
   The vote to release Apache NuttX 12.1.0-rc0 is now closed.
   Thanks to those that took the time to review and vote.
   
   The release has passed with 4 +1 (binding) votes and no 0 or -1 votes.
   
   Binding:
   +1 Lup Yuen Lee
   +1 Roberto Bucher
   +1 Tomek CEDRO
   +1 Alin Jerpelea
   
   Non binding
   +1 Filipe Cavalcanti
   
   Vote thread
   https://lists.apache.org/thread.html/r75faed90e03c7e7a07ff79988bb0586eec224905144f34e99333e9cd%40%3Cgeneral..apache.org%3E
   
   We will proceed with the official release of 12.1.0.

If the vote does not pass bring the feedback to the community and start the
release process again with a new RC.

Staging the release
===================

With the release approved you can now copy the release artifacts to the release
repository. Note it no longer has an RC in the folder name.

.. code-block:: console

   ~/nuttx/svn 
   $ cp -r nuttx-dev/12.1.0-RC0 nuttx-release/12.1.0
   
   ~/nuttx/svn 
   $ cd nuttx-release/
   
   ~/nuttx/svn/nuttx-release 
   $ svn status
   ?       12.1.0
   
   ~/nuttx/svn/nuttx-release 
   $ svn add 12.1.0
   A         12.1.0
   A  (bin)  12.1.0/apache-nuttx-12.1.0.tar.gz
   A  (bin)  12.1.0/apache-nuttx-apps-12.1.0.tar.gz
   A         12.1.0/apache-nuttx-12.1.0.tar.gz.sha512
   A         12.1.0/apache-nuttx-apps-12.1.0.tar.gz.sha512
   A  (bin)  12.1.0/apache-nuttx-12.1.0.tar.gz.asc
   A  (bin)  12.1.0/apache-nuttx-apps-12.1.0.tar.gz.asc
   $ svn commit -m "Releasing apache-nuttx-12.1.0"

At this point you should see the release at
https://dist.apache.org/repos/dist/release/nuttx/

Create release tags
===================

Create non RC tags the same way it was done for the RC tags on both
repositories:

.. code-block:: console

   # Export the signing key
   $ export GPG_TTY=$(tty)
   
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git checkout releases/12.1
   Already on 'releases/12.1'
   Your branch is up to date with 'origin/releases/12.1'.
   
   # Make sure it is up-to-date with upstream
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git pull
   Already up to date.
   
   # Make create the signed tag (note the -s option)
   ~/nuttx/wrk/nuttx on  releases/12.1 [$] 
   $ git tag -s nuttx-12.1.0 -m nuttx-12.1.0
   
   # Check that botht the RC and non RC tags exist on the commit
   ~/nuttx/wrk/release/nuttx on  releases/12.1 [$] took 4s 
   $ git log -n 1
   commit 16748108c503d762779545d40113825e54b75252 (HEAD -> releases/12.1, tag: nuttx-12.1.0-RC0, tag: nuttx-12.1.0, origin/releases/12.1)
   Author: Dong Heng <dongheng@espressif.com>
   Date:   Fri Apr 9 20:03:24 2021 +0800
   
       riscv/esp32c3: Fix heap end address
   
   # Push the tag
   ~/nuttx/wrk/release/nuttx on  releases/12.1 [$] 
   $  git push -u origin nuttx-12.1.0
   Enumerating objects: 1, done.
   Counting objects: 100% (1/1), done.
   Writing objects: 100% (1/1), 805 bytes | 402.00 KiB/s, done.
   Total 1 (delta 0), reused 0 (delta 0), pack-reused 0
   To github.com:apache/nuttx.git
    * [new tag]               nuttx-12.1.0 -> nuttx-12.1.0

You should be able to see the tag here https://github.com/apache/nuttx/tags and
https://github.com/apache/nuttx-apps/tags

Create a PR to add the Release to the Website
=============================================

This should include the release notes and also the metadata for downloading the
release. An example of this is here apache/nuttx-website#39 After 48hrs from
committing to the release SVN the distribution mirrors should have synced and
this can now be merged.

10min or so after the merge you should see the release here
https://nuttx.apache.org/download/

Send the release email out
==========================

Once the website shows the release you can now send the release announcement
out. Here is an example of that email. Note we must wait 48hr after the SVN
commit before sending this.

.. code-block:: text

   Subject: [ANNOUNCE] Apache NuttX 12.1.0 released
   To: dev@nuttx.apache.org

   The Apache NuttX project team is proud to announce
   Apache NuttX 12.1.0 has been released.

   The release artifacts and Release Notes can be found at:
   https://nuttx.apache.org/download/
   https://nuttx.apache.org/releases/12.1.0/

   Thanks,
   <Release Manager>
   on behalf of Apache NuttX PPMC
