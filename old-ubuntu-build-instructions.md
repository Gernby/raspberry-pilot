## MicroSD card preparation and first login

1. Please download Ubuntu 18.04.5 for the Raspberry Pi here http://cdimage.ubuntu.com/releases/18.04/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz on the laptop that will prepare the microSD card
2. Insert the microSD card into the microSD-to-SD card adapter and then into the computer you will use to burn the image
3. If you have Balena Etcher, flash the SD card with the image you just downloaded
4. If you are not using Balena Etcher, decompress the `xz`-compressed Ubuntu image file
5. Umount ("eject") the microSD card if the computer automatically mounted it but do not remove it from the slot
6. Burn the decompressed image to the microSD card (process varies by Operating System)

## Network preparation for the first boot

1. With the image written to the card, remove and reinsert the SD card adapter. Wait for the Operating System to detect and mount the microSD card.
2. Navigate to the "system-boot" partition using graphical or command line utilities
3. Locate the file called `user-data` and in the password expiry section, change `expire: true` to read `expire: false`
4. Save and exit the file

**Note: The easiest networking option is just plug the Pi into an RJ-45 port on your router or an unused port on a laptop that is using WiFi. Modern Operating Systems make it easy to share your WiFi connection with the Pi this way.**

If you cannot connect your Pi via a physical Ethernet cable, follow the steps below to setup WiFi on your Pi prior to first boot.

1. Open the file called `network-config` for editing.
2. Edit the lower portion of the file to resemble this example:

```
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      <replace with your home ssd>:
        password: "<enter your home WiFi WPA2 password>"
      <replace with your cellular hotspot ssid>:
        password: "<enter your hotspot WPA2 password>"
#     workssid:
#       auth:
#         key-management: eap
#         method: peap
#         identity: "me@example.com"
#         password: "passw0rd"
#         ca-certificate: /etc/my_ca.pem
```
3. Save and exit the file.

## First boot

1. Safely unmount the microSD card. Do not proceed until you know you have safely unmounted.
2. Remove the SD card adapter from the computer and remove the microSD card from the adapter
3. Insert the microSD card into the Rasperry Pi 4 with the contacts facing "up" towards the bottom of the mainboard
4. If you are using a local console, connect the keyboard, micro-HDMI adapter and the HDMI cable
5. Connect the Ethernet cable if you do not have WiFi.
6. Connect the Pi to a high-power USB port via the USB A-to-C cable.
7. Allow the Pi to boot and wait at least two minutes. If you are using the console, wait for several lines of text to appear before attempting to log in.
8. Locate the Pi IP address in your home WiFi router or your laptop if you are using your laptop to provide a network connection to the Pi
9. Log into the Pi using "ubuntu" for the username and the password
10. Try to `ping 8.8.8.8`. If successful, continue to the next section. If not, reboot and log in again with ubuntu/ubuntu.
11. If you still need to setup WiFi for use in the car, use the `nmcli` command to configure all of the WiFi access points you want to use in the car (home WiFi, cellular hotsopt). Run this command only once for every WiFi network name you plan to use only if you did not configure the WiFi prior to the first boot:

`sudo nmcli d wifi connect <SSID> password <password>`

## Software installation
(Note: You must begin the install within about 5 minutes. If you boot up the Pi but don't start the install relatively soon, the Pi will begin to update itself, preventing you from starting the install for about 20 minutes.)

1. Log into the Pi using "ubuntu" as the ID and password if you are not still logged in from earlier steps. Clone the repository
```
git clone https://github.com/Gernby/raspberry-pilot.git
mv raspberry-pilot/start_install_tf.sh .
sh start_install_tf.sh
```
2. Install should take just under 3 hours. If it finishes too quickly, come to Discord to discuss.
3. If the process completes successfully, reboot the Pi and log back in as the "ubuntu" user
4. Run the command `top -u ubuntu`
5. Look for `controlsd`, `boardd`, `ubloxd`, `transcoderd`, and `dashboard` in the rightmost column of the list.
6. If you see all five, you are ready to flash your Panda and go for a drive.
