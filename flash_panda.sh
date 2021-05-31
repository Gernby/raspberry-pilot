#!/bin/bash
echo "" 
echo "" 
echo -e "\e[44m***** This should work for all colors of Panda's, but only the Black Panda can be done with a single USB cable\e[0m"
echo -e "\e[44m***** White and Gray pandas require the RPi to be powered separately, while the Panda is connected via USB-A to USB-A cable\e[0m"
echo "" 
echo ""
echo ""
echo -e "\e[44m***** If flashing White Or Gray, please disconnect it from the Pi from the Panda now.\e[0m"
sleep 5
echo " Plug in the panda in 5 seconds!"
sleep 1
echo " Plug in the panda in 4 seconds!"
sleep 1
echo " Plug in the panda in 3 seconds!"
sleep 1
echo " Plug in the panda in 2 seconds!"
sleep 1
echo " Plug in the panda in 1 seconds!"
sleep 1
echo " Plug in the panda NOW!"
sleep 2
echo ""
pkill -f boardd
cd ~/raspilot/panda/board
#make clean
PYTHONPATH=~/raspilot make $1
echo ""
echo ""
echo -e "\e[44m***** if the panda is now flashing red slowly, it was successful\e[0m"
echo ""
echo -e "\e[44m***** If not, try running this again using the 'recover' option\e[0m"
echo -e "       \e[44mie. bash flash_panda.sh recover\e[0m"
echo ""