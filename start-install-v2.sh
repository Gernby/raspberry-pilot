# launch with

# sh start-install-v2 <wifi ssid> <wifi password>

# Disable IPv6 preference for apt
# uncomment

# precedence ::ffff:0:0/96 100

# in /etc/gai.conf to unstick IPv6 for apt

#sudo sed -i -e 's/\#precedence ::ffff:0:0\/96  100/precedence ::ffff:0:0  100/' /etc/gai.conf

# Install WiFi support and connect to WiFi using the supplied parameters
#sudo dpkg -i /var/cache/apt/archives/*deb
#sudo systemctl start NetworkManager

#sleep 30

#sudo nmcli d wifi connect $1 password $2

# Delete IPv6 from the current DHCP lease (not permanent)
#sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
#sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1
#sudo sysctl -w net.ipv6.conf.lo.disable_ipv6=1

# Delete the boot-up warning about not being able to check for updates
sudo rm /var/lib/ubuntu-release-upgrader/release-upgrade-available

# Install Ansible to support the rest of the installation
sudo apt-add-repository -y ppa:ansible/ansible
sudo apt update
sudo apt install ansible -y

# Start using Ansible from here

# Install Network Manager
# sudo apt install network-manager -y
# sudo service NetworkManager start

echo "Installing Network Manager for better WiFi management.."
ansible localhost -b -m apt -a "name=network-manager"
echo "Starting Netork Manager.."
ansible localhost -b -m service -a "name=NetworkManager started=yes"

# Disable IPv6 on future reboots

#ansible localhost -b -m lineinfile -a "path=/etc/sysctl.conf insertafter=EOF line=net.ipv6.conf.all.disable_ipv6=1"
#ansible localhost -b -m lineinfile -a "path=/etc/sysctl.conf insertafter=EOF line=net.ipv6.conf.default.disable_ipv6=1"
#ansible localhost -b -m lineinfile -a "path=/etc/sysctl.conf insertafter=EOF line=net.ipv6.conf.lo.disable_ipv6=1"

# Original start of start_install.sh after the network was configured

echo "Uninstall unattended-upgrades..."
ansible localhost -b -m apt -a "name=unattended-upgrades state=absent"

# Fixing permissions

echo "Fix ansible permissions in home directory.."
sudo chown -R ubuntu /home/ubuntu/.ansible

# Full upgrade

echo "Kicking off a full upgrade.. this could take 15 minutes."

#ansible localhost -v -b -m apt -a "upgrade=full"
ansible localhost -v -b -m apt -a "autoremove=yes"

# Run the playbook called "Install dependencies" in the original install script

echo "Installing the initial dependencies.."

#sudo apt install -y build-essential make automake python3.7-dev python3-pip
#sudo apt install -y openjdk-8-jdk automake autoconf zip unzip libtool swig libpng-dev zlib1g-dev pkg-config
#sudo apt install -y libhdf5-dev bzip2 clang git libarchive-dev
#sudo apt install -y libffi-dev libglib2.0-0 libssl-dev libswscale-dev
#sudo apt install -y libusb-1.0-0 libzmq5-dev ocl-icd-libopencl1 ocl-icd-opencl-dev
#sudo apt install -y opencl-headers wget checkinstall
#sudo apt install -y libatlas-base-dev libopenblas-base libopenblas-dev gcc gfortran
#sudo apt install -y capnproto autotools-dev uuid-dev libsodium-dev valgrind
#sudo apt install -y libusb-dev cmake libnewlib-arm-none-eabi libhdf5-serial-dev hdf5-tools smbclient
#sudo apt install -y influxdb apt-transport-https software-properties-common adduser libfontconfig1

#ansible localhost -b -m apt -a "name=['build-essential', 'make', 'automake', 'python3.7-dev', 'python3-pip']"
#ansible localhost -b -m apt -a "name=['openjdk-8-jdk', 'automake', 'autoconf', 'zip', 'unzip', 'libtool', 'swig', 'libpng-dev', 'zlib1g-dev', 'pkg-config']"
#ansible localhost -b -m apt -a "name=['libhdf5-dev', 'bzip2', 'clang', 'git', 'libarchive-dev']"
#ansible localhost -b -m apt -a "name=['libffi-dev', 'libglib2.0-0', 'libssl-dev', 'libswscale-dev']"
#ansible localhost -b -m apt -a "name=['libusb-1.0-0', 'libzmq5-dev', 'ocl-icd-libopencl1', 'ocl-icd-opencl-dev']"
#ansible localhost -b -m apt -a "name=['opencl-headers', 'wget', 'checkinstall']"
#ansible localhost -b -m apt -a "name=['libatlas-base-dev', 'libopenblas-base', 'libopenblas-dev', 'gcc', 'gfortran']"
#ansible localhost -b -m apt -a "name=['capnproto', 'autotools-dev', 'uuid-dev', 'libsodium-dev', 'valgrind']"
#ansible localhost -b -m apt -a "name=['libusb-dev', 'cmake', 'libnewlib-arm-none-eabi', 'libhdf5-serial-dev', 'hdf5-tools', 'smbclient']"
#ansible localhost -b -m apt -a "name=['influxdb', 'apt-transport-https', 'software-properties-common', 'adduser', 'libfontconfig1']"

ansible-playbook -v -b install-dependencies.yml

#      state: present

# Packages no longer available
# clang-3.8
# libusb-1.0

# Grafana

echo "Installing the grafana server..."
echo "Install grafana gpg signing key..."
ansible localhost -v -b -m apt_key -a "file=grafana.gpg.key"

echo "Add grafana repo..."
ansible localhost -v -b -m apt_repository -a "repo='deb https://packages.grafana.com/oss/deb stable main'"
echo "Install grafana server.."
ansible localhost -v -b -m apt -a "name=grafana update_cache=yes"
#ansible localhost -b -m apt -a "deb=grafana_6.6.2_arm64.deb"
echo "Prepare to start grafana.."
ansible localhost -v -b -m systemd -a "daemon-reload=yes"
echo "Configure the grafana server to start on boot.."
ansible localhost -v -b -m systemd -a "name=grafana-server enabled=yes"
echo "Start the InfluxDB server..."
ansible localhost -v -b -m service -a "name=influxdb state=started"
echo "Start the grafana server now"
ansible localhost -v -b -m service -a "name=grafana-server state=started"

echo "Install pipenv into user's home directory.."
ansible localhost -v -m pip -a "executable=/usr/bin/pip3 name=pipenv extra_args=--user"

echo "Cloning the repository..."

ansible localhost -v -m git -a "clone=yes repo=https://github.com/Gernby/raspberry-pilot.git version=next-breakthrough dest=/home/ubuntu/raspberry-pilot"
#git clone https://github.com/Gernby/raspberry-pilot.git -b fingerprints /home/ubuntu/raspberry-pilot

echo "Copy the finish_install script to homedir..."
ansible localhost -m copy -a "src=/home/ubuntu/raspberry-pilot/finish_install.sh dest=/home/ubuntu/"

echo "Reboot the Pi (run 'sudo reboot'), log back in, and run 'sh finish_install.sh' to complete the setup."

