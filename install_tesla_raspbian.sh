# Add parameter for Panda
tee /etc/udev/rules.d/11-panda.rules <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddcc", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="bbaa", ATTRS{idProduct}=="ddee", MODE="0666"
SUBSYSTEMS=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddcc", MODE:="0666"
SUBSYSTEMS=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddee", MODE:="0666"
EOF
udevadm control --reload-rules
udevadm trigger

# Add system boot parameters for Pi 4 and for PICAN
echo "dtoverlay=dwc2,dr_mode=host" | tee -a /boot/firmware/config.txt
echo "dtparam=spi=on" | tee -a /boot/firmware/config.txt
echo "dtoverlay=mcp251xfd,spi0-0,interrupt=25" | tee -a /boot/firmware/config.txt
echo " dwc_otg.lpm_enable=0 dwc_otg.otg_cap=1 modules-load=dwc2" | tee -a /boot/firmware/cmdline.txt

# Create final install script for 2nd boot
tee /home/raspilot/finish_install.sh <<EOF
apt update
apt install -y can-utils network-manager net-tools python3-full python3-pip libusb-1.0-0 libusb-1.0-0-dev influxdb influxdb-client vim htop
git clone https://github.com/gernby/raspberry-pilot /home/raspilot/raspilot
python3 -m venv /home/raspilot/raspilot
/home/raspilot/raspilot/bin/python3 -m pip install -U pip
/home/raspilot/raspilot/bin/python3 -m pip install setproctitle pandacan psutil influxdb python-can
rm -r /home/raspilot/raspilot/panda
chown -R raspilot.raspilot /home/raspilot/raspilot
crontab -r
su --login raspilot -c '(crontab -l; echo "@reboot bash raspilot/launch_tesla.sh";) | crontab -'
sudo reboot
EOF

# After reboot, wait 30 seconds for wifi to connect, then kick off the remaining install
(crontab -l; echo "@reboot sleep 30; bash /home/raspilot/finish_install.sh";) | crontab -
