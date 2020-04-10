#!/usr/bin/env bash

# We initialize the WiFi adapter for sending video
ifconfig wlan1 down                            # Bring the network interface down
iw dev wlan1 set type monitor                  # Enable monitor mode
iw dev wlan1 set monitor otherbss fcsfail      # Set monitor mode parameters
ifconfig wlan1 up                              # Bring the network inteface up
iwconfig wlan1 channel 13                      # Make the adapter send data through the given WiFi channel

# We set the "cpuset shield", whose purpose is to isolate the cores we specify from
# being taken by other processes
cset shield --cpu 3         # Only shield the last CPU
cset shield --kthread on    # Avoid certain kernel threads from scheduling here (not all of them)

# Finally, we execute the process on the last CPU
exec cset shield --exec /opt/icaros/icaros
