#!/bin/sh
# Give the Mac's wired Ethernet port a fixed address for nexys3_top_lan.
# Usage: tools/mac_lan_setup.sh                 (list network services)
#        tools/mac_lan_setup.sh "USB 10/100/1000 LAN"   (set 192.168.10.1/24)
#        tools/mac_lan_setup.sh "USB 10/100/1000 LAN" dhcp   (undo)
if [ -z "$1" ]; then
    echo "Network services on this Mac (pick the wired / USB LAN one):"
    networksetup -listallhardwareports
    exit 0
fi
if [ "$2" = "dhcp" ]; then
    sudo networksetup -setdhcp "$1"
else
    sudo networksetup -setmanual "$1" 192.168.10.1 255.255.255.0
fi
networksetup -getinfo "$1"
