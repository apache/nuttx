# The Inviolable Principles of NuttX

These are properties of NuttX that we can be certain of for all time:

## Definition

*in·vi·o·la·ble*
/inˈvīələbəl/

adjective
adjective: inviolable

    never to be broken, infringed, or dishonored.

    "an inviolable rule of chastity"

    synonyms:  inalienable, absolute, untouchable, unalterable,
               unchallengeable, unbreakable, impregnable; sacrosanct,
               sacred, holy, hallowed; rare intemerate

    "the inviolable right to life"

Source: Oxford Dictionary of the English Language

## Strict POSIX compliance

  - Strict conformance to the portable standard OS interface as defined at
    OpenGroup.org.
  - A deeply embedded system requires some special support.  Special
    support must be minimized.
  - The portable interface must never be compromised only for the sake of
    expediency.
  - Expediency or even improved performance are not justifications for
    violation of the strict POSIX interface.

## Modular Architecture

  - The internal modular architecture of the OS must be maintained.
  - This means formalizing and documenting all internal interfaces (in the
    porting guide), minimal use of global variables at the interface, and
    only well defined functional interfaces.

## Clear, Consistent, Standardized Coding Style

  - Strict conformance to the NuttX coding style.  No "revolutionary"
    changes to the coding standard (but perhaps some "evolutionary"
    changes).
  - Personal or organizational preference is not a justification for a
    coding style change.
  - Nothing can come into NuttX that does not follow the coding standard.
  - Expediency is not a justification for violating the coding standard.

  The NuttX coding standard can be found here:
  https://nuttx.apache.org/docs/latest/contributing/coding_style.html

## Open and Unencumbered License

  - Currently BSD 3-clause or compatible:  BSD 3-clause with constraints,
    BSD 3 and 4 clause, MIT, public domain.
  - Other unencumbered licenses such as Apache may be considered.
    NuttX will never be licensed under a restrictive, "Copyleft" license.

## All Users Matter

  - All support must apply equally to all supported platforms.  At present
    this includes Linux, Windows MSYS, Windows Cygwin, Windows Ubuntu,
    Windows native, macOS, Solaris, and FreeBSD.  No tool/environment
    solutions will be considered that limit the usage of NuttX on any of
    the supported platforms.
  - Inclusive rather than exclusive.
  - Hobbyists are valued users of the OS including retro computing hobbyists
    and DIY “Maker” hobbyists.
  - Supported toolchains:  GCC, Clang, SDCC, ZiLOG ZDS-II (c89), IAR.
    Others?
  - No changes to build system should limit use of NuttX by any user.
  - Simplifying things for one user does not justify excluding another user.
  - We should seek to expand the NuttX user base, not to limit it for
    reasons of preference or priority.
  - We must resist the pull to make NuttX into a Linux-only, GCC-only, and
    ARM-only solution.

## NuttX Branding

  - The official name of authentic NuttX will always be "NuttX".
  - This name is trademarked and may not be used by other OSs or forks of
    NuttX.

## The Enemies

### No Short Cuts

  - Doing things the easy way instead of the correct way.
  - Reducing effort at the expense of Quality, Portability, or
    Consistency.
  - Focus on the values of the organization, not the values of the Open
    Source project.  Need to support both.
  - It takes work to support the Inviolables.  There are no shortcuts.

### Sometimes Code Duplication is OK

  - Sometimes is better to duplicate some logic than to introduce coupling.

### Keep the Big Picture

  - Too much focus on solving the problem in hand, loss of the Big Picture.
  - Insufficient understanding of the architectural principles.

### Conform to Standards

  - Changing things only to suit a personal or organizational preference.
  - Inflexibility, Inability to adapt.
  - Not Invented Here (NIH) syndrome.
