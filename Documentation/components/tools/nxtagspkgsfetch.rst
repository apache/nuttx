======================
``nxtagspkgsfetch.sh``
======================

This script downloads all NuttX RTOS and Application snapshot packages
from the upstream git repository based on the provided git tags list.
These are NOT official release packages as checksum will differ.
When launched from the local NuttX git repository clone the script will
obtain all available tags to be downloaded, otherwise list of tags needs
to be provided manually (or when just selected tag is required).
This script uses ``wget`` underneath, make sure this tool is installed.
Fetch log file is created with a timestamp in name next to the packages.

Having all tags packaged is important for changes comparison
between specific versions, testing a specific version, compatibility
checks, searching for a feature introduction timeline, etc.

Usage: ``./nxtagspkgsfetch.sh [download_path] [tags_list_space_separated]``

You can provide optional download path (default ``../../nuttx-packages``)
and tags list to get packages for (default all tags from local git clone).
When providing tags you also need to provide download path.
