#!/bin/bash

# this script is meant to run on a fresh image of 64-bit Ubuntu Server for ARM, specifically Raspberry Pi 4.
# However, I think it should work for any ARM64 SBC on 64-bit Ubuntu for ARM.
#
# NOTE: A fresh image has unattended-upgrades enabled. You must wait
# for the initial upgrade to complete before running this script
#
# Download the image for RPi4 here
# https://ubuntu.com/download/raspberry-pi/thank-you?version=18.04.4&architecture=arm64+raspi3
#
# For other ARM64, download from here.
# http://cdimage.ubuntu.com/releases/18.04.4/release/ubuntu-18.04.4-server-arm64.iso
#
# create flash using something like Balena Etcher
# https://www.balena.io/etcher/
#
# In Linux:
# xzcat ubuntu-18.04.4-preinstalled-server-arm64+raspi3.img.xz | sudo dd bs=4M of=/dev/blkmmc0
#
# connect to LAN, monitor and keyboard
# change default password for ubuntu (default login is ubuntu / ubuntu)
# run ifconfig to get the Pi IP address
# SSH into RPi
# ssh ubuntu@<ip address>
#
# connect to Wifi
# sudo apt install network-manager
# systemctl start NetworkManager
# nmcli d wifi list
# sudo nmcli d wifi connect <WiFiSSID> password <WiFiPassword>
# 
# monitor CPU temperature
# cat /sys/class/thermal/thermal_zone0/temp
# 42355 = 42.355 C

# update software from repository
sudo apt update

# disable unattended upgrades
sudo apt remove -y unattended-upgrades

# update Ubuntu and clean up
sudo apt full-upgrade -y
sudo apt autoremove -y

# install dependencies
sudo apt install -y build-essential make automake python3-numpy python3-dev python3-pip python3-mock python3-scipy
sudo apt install -y openjdk-8-jdk automake autoconf zip unzip libtool swig libpng-dev zlib1g-dev pkg-config
sudo apt install -y libhdf5-dev bzip2 clang git libarchive-dev libavcodec-dev libavdevice-dev libavfilter-dev 
sudo apt install -y libavresample-dev libavutil-dev libffi-dev libglib2.0-0 libssl-dev libswscale-dev
sudo apt install -y libtool libusb-1.0-0 libzmq5-dev ocl-icd-libopencl1 ocl-icd-opencl-dev 
sudo apt install -y opencl-headers pkg-config python-pip wget checkinstall libusb-1.0
sudo apt install -y clang-3.8 libatlas-base-dev libopenblas-base libopenblas-dev gcc gfortran ocl-icd-opencl-dev 
sudo apt install -y capnproto opencl-headers autotools-dev uuid-dev libsodium-dev valgrind python-qt4
sudo apt install -y libusb-dev cmake libnewlib-arm-none-eabi libhdf5-serial-dev hdf5-tools smbclient
sudo apt install -y influxdb apt-transport-https software-properties-common adduser libfontconfig1
wget https://dl.grafana.com/oss/release/grafana_6.6.2_arm64.deb
sudo dpkg -i grafana_6.6.2_arm64.deb
sudo /bin/systemctl daemon-reload
sudo /bin/systemctl enable grafana-server
sudo service influxdb start
sudo service grafana-server start

pip3 install pipenv --user
cp ~/raspberry-pilot/finish_install.sh ~/
# Exit SSH and log in again, since pipenv won't work without it
