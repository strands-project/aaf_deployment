#!/bin/bash

if [ -z "$1" ]; then
	echo "usage: $0 <user:password> <destionation-dir> <FILES...>" >&1
	exit 2
fi

LOGIN="$1"
shift
DESTDIR="$1"
shift
URL_BASE="https://lcas.lincoln.ac.uk/owncloud/remote.php/webdav/"


for f in "$@"; do
	bn=`basename "$f"`
	curl -X PUT --progress-bar --verbose --user $LOGIN --data-binary "@$f" "$URL_BASE/$DESTDIR/$bn"
done