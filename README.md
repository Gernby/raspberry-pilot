# Raspberry Pilot Installation Steps

## Requirements

1. A laptop capable of burning microSD cards and has a decent battery life (min. 30 minutes)
2. Compression software on the above laptop that can decompress the `xz` format (titles and installation process varies by Operating System)
3. Raspberry Pi 4 with 4GB RAM
4. White, Gray, or Black Panda from Comma.ai shop
5. Honda Bosch Giraffe from Comma.ai shop if using a White or Gray Panda, Honda Bosch harness for Black Panda
6. USB A-to-C cable ([This](https://www.amazon.com/Anker-2-Pack-Premium-Charging-Samsung/dp/B07DC5PPFV) two pack of 6-ft. cables is popular)
7. USB A-to-A cable (like [this](https://www.amazon.com/gp/product/B0002MKBI2) one) or mini USB cable with a Panda Paw from the Comma.ai shop (for flashing White or Gray Pandas from the Pi)
8. A way to power the Pi in your car (battery pack, laptop, or high power 12v power adapter for the car)
9. Minimum 16GB A1-rated microSD card
10. Optional Micro-HDMI adapter plus TV or monitor that accepts HDMI input (the Raspberry Pi 4 does not use the same HDMI connectors as any previous models)
11. Optional USB keyboard
12. A physical Ethernet connection is optional but works best during the install phase
13. Cellular hotspot or home WiFi you can reach from your car (no cable company or retail WiFi)

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

## Flashing the Panda

**Note: If you can't hit your household WiFi from your car, be sure to configure the Pi to hit your cellular hotspot and turn on your hotspot before booting up the Pi in the car.**

### Black Panda

1. Connect the Pi to the Black Panda using the standard configuration (USB A on the Panda to USB C on the Pi)
2. Turn on the car
3. SSH into the Pi
4. Edit `~/kegman.json` and change the value in `useAutoFlash` from `0` to `1`.
5. Turn the car off and unplug the Pi.

### White or Gray Panda

1. Before taking the Pi outside, edit `~/kegman.json` and change the value in `useAutoFlash` from `0` to `1`. Safely shut down the Pi.
2. Take the Pi outside with a separate power source, a USB C cable to power the Pi, and the USB A-to-A cable for flashing the Panda
3. Connect the USB A-to-A cable between the Pi and the Panda
4. Power up the Pi and wait two minutes. The Panda should cycle through several colors, before ending in a slow pulsing color or combination of colors. If it the process ends in a fast flashing green LED, it was not successful.
5. Unplug the power supply from the Pi. Remove the USB A-to-A cable from the Pi and the Panda.

If you have successfully flashed the Panda, you are ready to calibrate and go for your first drive. If you are unable to flash the Panda or are not convinced that you have, come to Discord to discuss the issue. As a reminder, if you flashed a White or Gray Panda, you will need to reconfigure your setup to back to the standard configuration: the Panda is connected to the Pi via the USB A-to-C cable, with the USB A end in the Panda and the USB C end in the power port on the Pi. The USB A-to-A cable is not used during standard operation.

## First Drive and Training

1. With the output of the `top` command verified, the Panda flashed and the Pi and Panda connected in the standard configuration, you should be ready to train the software and drive
2. Turn on the car and watch the dash. The `ACC` and `LKAS` indicators should only appear Orange for a couple of seconds before turning Green.
3. After about a minute, the lane marking indicators should light up but show as outlines. You do not need to wait for this to show up before driving.
4. Drive to a road with well-marked lines on both sides of the car and with minimal curves and breaks in the lines -- an interstate is preferred
5. Drive the car on the interstate for about 5 miles. You must hold the wheel as steady as you can and hold the car in the center of the lane as much as possible. This training period is critical to the future performance of the software.
6. After about 5 miles, Raspberry Pilot should take over steering automatically and without any input from you. If RP does not start steering, something went wrong and you need to keep your hands on the wheel until you can stop and troubleshoot.
7. Keep your hands lightly brushing the wheel and be ready to take over if at any point you feel uncomfortable about the way the software is steering
8. If you are completely uncomfortable, press the Lane Keeping Assist button beneath the "SET" button in the ACC cruise control section of the steering wheel to disable the Raspberry Pilot LKAS.
9. On all subsequent drives, Raspberry Pilot will start steering the car about 75 seconds after turning on the car. Again, press the LKAS button any time you do not want this feature enabled.

## Notes

The ACC radar cruise control functionality is 100% stock; it remains engaged after a press of the gas and it disengages immediately upon a press of the brakes

Raspberry Pilot defaults to steering 100% of the time, even when the radar cruise (ACC) is not enabled. You may press the Lane Keeping Assist button on the steering wheel to toggle the LKAS functionality on and off at any time. It has a picture of a steering wheel centered between lane lines on a highway.
