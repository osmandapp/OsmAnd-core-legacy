#!/bin/bash
if [ ! -n "$OSMAND_TARGET_PREFIX" ]
then
    SNAME=`basename $0`
    OSMAND_TARGET_PREFIX=${SNAME%.*}
fi
OSMAND_TARGET_PREFIX="$OSMAND_TARGET_PREFIX" OSMAND_TARGET=arm64-macosx-clang `dirname $0`/.cmake/utils/makefile.sh "$@"
