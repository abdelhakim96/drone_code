#!/bin/bash
echo "Starting procedure ..."
echo "Turning off power safe mode for better wifi connection..."    
sudo iw dev wlan0 set power_save off
echo "DONE"    
echo "Time server synchronizing..."  
sudo service ntp stop
sudo ntpdate -s 0.dk.pool.ntp.org
sudo service ntp start
ntpdate -q 0.dk.pool.ntp.org
echo "DONE"
echo "Startup Procedure finished"
