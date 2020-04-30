#!/bin/sh

FILELIST=`ls -1 up_*.S`

for file in $FILELIST; do
  newname=`echo $file | sed -e "s/up_/arm_/g"`
  echo "### $file->$newname"
  git mv $file $newname

#  oldbase=`basename $file`
#  newbase=`basename $newname`
#  sed -i -e "s/${oldbase}/${newbase}/g" $file
done

