#!/bin/bash

if ! /usr/bin/nmcli c status id "STRANDS AAF" > /dev/null; then
	echo "connection dropped and needs to be re-initiated"
	sudo /usr/bin/nmcli nm wifi off
	sleep 5
        sudo /usr/bin/nmcli nm wifi on
	sleep 5
	sudo /usr/bin/nmcli c up id "STRANDS AAF"
 	#nmcli d wifi connect "STRANDS" password 12345678
fi
