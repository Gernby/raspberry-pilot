#!/bin/bash

# this script is meant to run on a fresh image for the Nvidia Jetson Nano.
#
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

git clone https://github.com/gernby/raspberry-pilot.git raspilot
python -m pip3 install pipenv --user

# Exit SSH and log in again, since pipenv won't work without it
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

sudo cp ~/raspilot/phonelibs/influxdb.conf /etc/influxdb/influxdb.conf

sudo apt --fix-broken install
sudo apt clean -y
sudo bash phonelibs/install_capnp.sh

pipenv install
pipenv shell
bash build_all.sh
