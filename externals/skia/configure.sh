#!/bin/bash

SRCLOC="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if ls -1 $SRCLOC/upstream.patched >/dev/null 2>&1
then
   echo "Exists $SRCLOC/upstream.patched, ignoring..."
   exit
fi

# Delete old one if such exists
if ls -1 $SRCLOC/upstream.* >/dev/null 2>&1
then
	echo "Deleting old upstream..."
	rm -rf $SRCLOC/upstream.*
fi

# Download
echo "Downloading new upstream..."
VERSION="android/11-release"
mkdir -p "$SRCLOC/upstream.original"
(cd "$SRCLOC/upstream.original" && \
	git init && \
	git remote add origin -t $VERSION https://github.com/google/skia.git && \
	git fetch --depth=1 && \
	git checkout $VERSION)

# Patch
cp -rf $SRCLOC/upstream.original $SRCLOC/upstream.patched


# get python
EXEC_PY=$(which python)
if [ ! -e "${EXEC_PY}" ]; then
  EXEC_PY=$(which python3)
fi

# sync deps
cd $SRCLOC/upstream.patched/tools
${EXEC_PY} ./git-sync-deps

VERSION_HARFBUZZ="b37f03f" #2.8.1
(cd $SRCLOC/upstream.patched/third_party/externals/harfbuzz && \
	git checkout $VERSION_HARFBUZZ)
	
#Patch jerror.c after git sync, before it is not available
#patch $SRCLOC/upstream.patched/third_party/externals/libjpeg-turbo/jerror.c $SRCLOC/patches/12-libjpeg-jerror.after_git_sync_patch
if [ -d $SRCLOC/patches ]; then
	echo "Patching..."
	PATCHES=`ls -1 $SRCLOC/patches/*.patch | sort`
	for PATCH in $PATCHES
	do
		read  -rd '' PATCH <<< "$PATCH"
		echo "Applying "`basename $PATCH`
		patch -l --strip=1 --directory=$SRCLOC/upstream.patched/ --input=$PATCH
	done
fi
