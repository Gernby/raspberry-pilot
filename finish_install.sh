#!/bin/bash

cd ~/
mv raspberry-pilot raspilot
cd raspilot

sudo mkdir /data
sudo mkdir /data/params
sudo chown ubuntu /data
sudo chown ubuntu /data/params
sudo tee /etc/udev/rules.d/11-panda.rules <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddee", MODE="0666"
SUBSYSTEMS=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddcc", MODE:="0666"
SUBSYSTEMS=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddee", MODE:="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

(crontab -l; echo "@reboot bash raspilot/launch_openpilot.sh";) | crontab -
crontab -l

sudo cp ~/raspilot/phonelibs/usercfg.txt /boot/firmware/usercfg.txt
sudo cp ~/raspilot/phonelibs/influxdb.conf /etc/influxdb/influxdb.conf

sudo apt --fix-broken install -y
sudo apt clean -y
sudo bash phonelibs/install_capnp.sh

pipenv install
pipenv run bash build_all.sh
