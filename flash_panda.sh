#!/bin/bash
echo "" 
echo "" 
echo -e "\e[44m***** This must be executed using pipenv (ie. cd ~/raspilot && pipenv run bash flash_panda.sh\e[0m"
echo -e "\e[44m***** the Panda and RPi should be separately powered, and connected via USB-A to USB-A cable\e[0m"
echo "" 
echo ""
echo ""
sleep 10
pkill -f boardd
cd ~/raspilot/panda/board
make clean
PYTHONPATH=~/raspilot make
echo ""
echo ""
echo -e "\e[44m***** If the first part of the flash succeeded, the Panda should be flashing green rapidly\e[0m"
sleep 5
echo -e "\e[44m***** Now unplug the USB-A cable from the RPi for 10 seconds\e[0m"
sleep 10
echo -e "\e[44m***** after 10 seconds, plug it back into a different USB port\e[0m"
echo ""
sleep 7
PYTHONPATH=~/raspilot make
echo ""
echo ""
echo -e "\e[44m***** if the panda is now flashing red slowly, it was successful\e[0m"
echo ""