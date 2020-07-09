#!/bin/bash

# brand new ubuntu install on SD card
# ssh or log into the Pi
# cd
# git clone -b TensorFlow-2-2 https://github.com/Gernby/raspberry-pilot.git
# mv ~/raspberry-pilot/start_install_tf.sh .
# sh start_install_tf.sh

# rename the folder
cd ~
mv ~/raspberry-pilot ~/raspilot
cd ~/raspilot

# add grafana signing key and repository
sudo apt-key add grafana.gpg.key
sudo add-apt-repository "deb https://packages.grafana.com/oss/deb stable main"

# update software from repository
sudo apt update

sudo apt install -y  network-manager
#ansible localhost -b -m lineinfile -a "path=/etc/default/crda regexp='^REGDOMAIN=' line='REGDOMAIN=US'"
sudo sed -i -e 's/REGDOMAIN=/REGDOMAIN=US/' /etc/default/crda
sudo systemctl start NetworkManager

# force rescan of the available wifi networks
# sudo nmcli dev wifi rescan

# connect to Wifi (these are optional parameters and won't block the script from running)
sudo nmcli d wifi connect $1 password $2

# disable unattended upgrades
sudo apt remove -y unattended-upgrades

# update Ubuntu and clean up
sudo apt full-upgrade -y
sudo apt autoremove -y

# install dependencies
sudo apt install -y build-essential make automake python3.7-dev python3-pip jq
sudo apt install -y openjdk-8-jdk automake autoconf zip unzip libtool swig libpng-dev zlib1g-dev pkg-config
sudo apt install -y libhdf5-dev bzip2 clang git libarchive-dev  
sudo apt install -y libffi-dev libglib2.0-0 libssl-dev libswscale-dev
sudo apt install -y libtool libusb-1.0-0 libzmq5-dev ocl-icd-libopencl1 ocl-icd-opencl-dev 
sudo apt install -y opencl-headers pkg-config wget checkinstall libusb-1.0
sudo apt install -y clang-3.8 libatlas-base-dev libopenblas-base libopenblas-dev gcc gfortran ocl-icd-opencl-dev 
sudo apt install -y capnproto opencl-headers autotools-dev uuid-dev libsodium-dev valgrind
sudo apt install -y libusb-dev cmake libnewlib-arm-none-eabi libhdf5-serial-dev hdf5-tools smbclient
sudo apt install -y influxdb influxdb-client apt-transport-https software-properties-common adduser libfontconfig1 dfu-util

# install and start grafana; reverse the comments below to perform a standard install of grafana
sudo apt install -y grafana
#wget https://dl.grafana.com/oss/release/grafana_6.6.2_arm64.deb
#sudo dpkg -i grafana_6.6.2_arm64.deb

sudo /bin/systemctl daemon-reload
#sudo /bin/systemctl enable grafana-server
#sudo service influxdb start
#sudo service grafana-server start

#python3 -m pip install pipenv --user

wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v2.2.0/tensorflow-2.2.0-cp37-none-linux_aarch64.whl 
python3.7 -m pip install cython
python3.7 -m pip install tensorflow-2.2.0-cp37-none-linux_aarch64.whl
python3.7 -m pip install sklearn
python3.7 -m pip install pyzmq
python3.7 -m pip install pycapnp 
python3.7 -m pip install setproctitle
python3.7 -m pip install cffi
python3.7 -m pip install cython

# folders for storing various hdf5 files 
mkdir ~/buttons
mkdir ~/buttons/model-1
mkdir ~/buttons/model-2
mkdir ~/buttons/model-3
mkdir ~/buttons/model-4

cd
cp ~/raspilot/finish_install.sh ~/
sh ~/finish_install.sh
# Exit SSH and log in again, since pipenv won't work without it
