#!/bin/bash

# make sure you configure /usr/bin/nmcli with the NOPASSWD option in sudoers:
#   ALL ALL=(ALL) NOPASSWD: /usr/bin/nmcli

ESSID="STRANDS AAF"

# check for connection status, option 1: ask nmcli
# if ! /usr/bin/nmcli c status id "$ESSID" > /dev/null; then

# check for connection status, option 2: check real connection
curl -D /tmp/headers-$USER.txt -s 'http://captive.apple.com/hotspot-detect' > /tmp/captive-return-$USER.html
http_code=`grep "HTTP/1.1" /tmp/headers-$USER.txt | cut -f2 -d" "`
if [ "$http_code" = "200" ]; then
    sleep 1
else
	echo "connection dropped and needs to be re-initiated"
	sudo /usr/bin/nmcli nm wifi off
	sleep 5
        sudo /usr/bin/nmcli nm wifi on
	sleep 5
	sudo /usr/bin/nmcli c up id "$ESSID"
fi

