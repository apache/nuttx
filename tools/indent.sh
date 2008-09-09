#!/bin/sh
#
# This script uses the Linux 'indent' utility to re-format C source files
# to match the coding style that I use.  It differs from my coding style in that
#
# - I normally put the traiing */ of a multi-line comment on a separate line,
# - I usually align things vertically (like '='in assignments.
#

# Constants

options="-nbad -bap -bbb -nbbo -nbc -bl -bl2 -bls -nbs -cbi2 -ncdw -nce -ci2 -cli0 -cp40 -ncs -nbfda -nbfde -di1 -nfc1 -fca -i2 -l80 -lp -ppi2 -lps -npcs -pmt -nprs -npsl -saf -sai -sbi2 -saw -sc -sob -nss -nut"

usage="USAGE: $0 <in-file> <out-file>"

# Inputs

infile=$1
outfile=$2

# Verify inputs

if [ -z "$infile" ]; then
    echo "Missing <in-file>"
    echo $usage
    exit 1
fi

if [ ! -r $infile ]; then
    echo "Readable $nfile does not exist"
    exit 1
fi

if [ -z "$outfile" ]; then
    echo "Missing <out-file>"
    echo $usage
    exit 1
fi

if [ -f $outfile ]; then
    echo "Removing old $outfile"
    rm $outfile || { echo "Failed to remove $outfile" ; exit 1 ; }
fi

# Perform the indentation

indent $options $infile -o $outfile


