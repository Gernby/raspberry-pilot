#!/bin/bash

# User instructions
# Start with a brand new ubuntu install on SD card
# plug in an Ethernet cable or get the WiFi working
# ssh or log into the Pi on a local console
# cd
# git clone -b TensorFlow-2-2 https://github.com/Gernby/raspberry-pilot.git
# mv ~/raspberry-pilot/start_install_tf.sh ~
# sh start_install_tf.sh

# every so often, please upload data logs for analysis using this command
# cd ~/raspilot && PYTHONPATH=~/raspilot python3.7 upload_files.py

# --- Install script starts here ---

# Install Ansible to support the future direction
sudo apt-add-repository -y ppa:ansible/ansible
sudo apt update
sudo apt install ansible -y

# rename the folder
cd ~
mv ~/raspberry-pilot ~/raspilot
cd ~/raspilot

# Add grafana signing key and repository
# sudo apt-key add grafana.gpg.key
# sudo add-apt-repository "deb https://packages.grafana.com/oss/deb stable main"

echo "Install grafana gpg signing key..."
ansible localhost -v -b -m apt_key -a "file=/home/ubuntu/raspilot/grafana.gpg.key"

echo "Add grafana repo..."
ansible localhost -v -b -m apt_repository -a "repo='deb https://packages.grafana.com/oss/deb stable main'"

echo "Install grafana server.."
ansible localhost -v -b -m apt -a "name=grafana update_cache=yes"
# sudo apt install grafana -y

# update software from repository
sudo apt update

# Install Network Manager
# sudo apt install network-manager -y && sudo service NetworkManager start

echo "Installing Network Manager for better WiFi management.."
# sudo apt install -y  network-manager
# sudo sed -i -e 's/REGDOMAIN=/REGDOMAIN=US/' /etc/default/crda
ansible localhost -b -m apt -a "name=network-manager"
ansible localhost -b -m lineinfile -a "path=/etc/default/crda regexp='^REGDOMAIN=' line='REGDOMAIN=US'"

echo "Starting Network Manager.."
# sudo systemctl start NetworkManager
ansible localhost -b -m service -a "name=NetworkManager state=started"

# force rescan of the available wifi networks
# sudo nmcli dev wifi rescan

# connect to Wifi (these are optional parameters and won't block the script from running)
sudo nmcli d wifi connect $1 password $2

# disable unattended upgrades
ansible localhost -b -m apt -a "name=unattended-upgrades state=absent"
# sudo apt remove -y unattended-upgrades

# update Ubuntu and clean up
echo "Updating Ubuntu and removing unneeded packages.."
#sudo apt full-upgrade -y
#sudo apt autoremove -y
ansible localhost -v -b -m apt -a "upgrade=full"
ansible localhost -v -b -m apt -a "autoremove=yes"

# install dependencies
echo "Installing dependencies.."
sudo apt install -y build-essential make python3.7-dev python3-pip libzmq3-dev python3-zmq
sudo apt install -y openjdk-8-jdk automake zip unzip libtool swig libpng-dev pkg-config
sudo apt install -y libhdf5-dev clang libarchive-dev  
sudo apt install -y libssl-dev libswscale-dev
sudo apt install -y libusb-1.0-0 libusb-1.0-0-dev ocl-icd-libopencl1 ocl-icd-opencl-dev 
sudo apt install -y opencl-headers checkinstall
sudo apt install -y clang-3.8 libatlas-base-dev libopenblas-base libopenblas-dev gfortran
sudo apt install -y capnproto uuid-dev libsodium-dev valgrind
sudo apt install -y libusb-dev cmake libnewlib-arm-none-eabi libhdf5-serial-dev hdf5-tools smbclient
sudo apt install -y influxdb influxdb-client apt-transport-https adduser dfu-util jq
# already installed:
# autoconf automake zlib1g-dev bzip2 git libffi-dev libglib2.0-0 libzmq5-dev wget gcc autotools-dev libfontconfig1 software-properties-common

# Ansible version for the future
#ansible localhost -b -m apt -a "name=build-essential,make,automake,python3.7-dev,python3-pip,libzmq3-dev,python3-zmq"
#ansible localhost -b -m apt -a "name=openjdk-8-jdk,automake,zip,unzip,libtool,swig,libpng-dev,pkg-config"
#ansible localhost -b -m apt -a "name=libhdf5-dev,clang,libarchive-dev"
#ansible localhost -b -m apt -a "name=libssl-dev,libswscale-dev"
#ansible localhost -b -m apt -a "name=libusb-1.0-0,libusb-1.0-0-dev,ocl-icd-libopencl1,ocl-icd-opencl-dev"
#ansible localhost -b -m apt -a "name=opencl-headers,checkinstall"
#ansible localhost -b -m apt -a "name=libatlas-base-dev,libopenblas-base,libopenblas-dev,gfortran"
#ansible localhost -b -m apt -a "name=capnproto,uuid-dev,libsodium-dev,valgrind"
#ansible localhost -b -m apt -a "name=libusb-dev,cmake,libnewlib-arm-none-eabi,libhdf5-serial-dev,hdf5-tools,smbclient"
#ansible localhost -b -m apt -a "name=influxdb,influxdb-client,apt-transport-https,adduser,dfu-util,jq"
# already installed:
# autoconf automake zlib1g-dev bzip2 git libffi-dev libglib2.0-0 libzmq5-dev wget gcc autotools-dev libfontconfig1 software-properties-common

# change default python
echo "Changing the default python.."
sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 2

# start grafana and influxdb (installed but temporarily disabled to reduce resource usage)
sudo /bin/systemctl daemon-reload
#sudo /bin/systemctl enable grafana-server
#sudo service influxdb start
#sudo service grafana-server start

# Install the TensorFlow 2.2 components and dependencies
wget https://github.com/lhelontra/tensorflow-on-arm/releases/download/v2.2.0/tensorflow-2.2.0-cp37-none-linux_aarch64.whl 
python3.7 -m pip install cython
python3.7 -m pip install tensorflow-2.2.0-cp37-none-linux_aarch64.whl
python3.7 -m pip install sklearn
python3.7 -m pip install pyzmq
python3.7 -m pip install pycapnp 
python3.7 -m pip install setproctitle
python3.7 -m pip install cffi
python3.7 -m pip install cython
python3.7 -m pip install pandacan

# Create folders for storing various hdf5 files; supports switching models via remote ssh commands
mkdir ~/buttons
mkdir ~/buttons/model-1
mkdir ~/buttons/model-2
mkdir ~/buttons/model-3
mkdir ~/buttons/model-4

# Kick off the final stage of the build
cd
cp ~/raspilot/finish_install.sh ~/
sh ~/finish_install.sh

echo "Please reboot, run top -u ubuntu, and look for boardd, dashboard, and controlsd"
