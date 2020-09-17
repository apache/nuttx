# Licensing Check Scripts

This directory holds various scripts to aid in clearing IP on files. The main script is
`log2json` which should receive a path to a file (relative to repository root). It will retrieve
the file history and build a JSON output with all metadata needed for analysis. The second
script is `check.py` which receives a JSON file generated from the previous tool (either from a file
or from stdin, using `-` for the filename).

The check script will:

  1. retrieve git commit authors
  2. parse commit message for possible attributions ("authored by: ...", among other variations)
  3. retrieve file contents at each commit, parse the license header and try to extract authors
     and companies (copyrights) listed there

Steps 2 and 3 are based on heuristics. The attributions may not match the regular expressions
used so there may be misdetections. Authors on headers are easier to detect. In fact, this will
pick up various false positives (non-author strings) which will have to be ignored by the user.

All of these authorship information is aggregated and in a final step, the names are used 
to check for ICLAs, based on the ICLA databases (see below), which need to be manually downloaded.
If a given author name is not matched, their email searched for in the `author_mappings.json` file,
which is a dictionary of email to real name. This allows to handle users with alternative email
addresses.

The script output will report a green check if author matched the ICLA database or a red cross
if not. Note that given the false positives in steps 2 and 3, there may be both non-author strings
that obviously do not match and also there may be an attribution which was not detected in a commit
message. The thorough approach would be to run the check script with verbosity ('-v') which will
print the metadata of each commit, including the commit message. If double verbosity is used ('-vv'),
the whole file will be printed, which allows to check the header.

## Inaccessible blobs

Since some files in the repositories lived during some part of their history in a separate repository
(linked as a submodule to main repo), their blobs (basically the file at a given point in time)
will not be accessible. This means that the file at that point in time cannot be accessed for analyzing
its header.

## Zero blob hash

Some blob hashes will be all zeros, which means that the file was deleted at this point in time.
Sometimes this is due to merges or renames (which may be part of the moving in and out of submodules).

## ICLA database

In order to retrieve a list of all users with CLAs,
download the following files:

  * https://whimsy.apache.org/public/icla-info.json
  * https://whimsy.apache.org/public/icla-info_noid.json

There are two files since not all users with CLAs have
Apache IDs. These lists do not contain emails, but a
manual search form is also here:

  * https://whimsy.apache.org/roster/committer/
