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
echo -e "\e[44m***** if the panda is now flashing red slowly, it was successful\e[0m"
echo ""