# Raspberry Pilot Installation Steps

## Requirements

1. A laptop capable of burning microSD cards and has a decent battery life (min. 30 minutes)
2. Compression software on the above laptop that can decompress the `xz` format (titles and installation process varies by Operating System)
3. Raspberry Pi 4 with 4GB RAM
4. White, Gray, or Black Panda from Comma.ai shop
5. Honda Bosch Giraffe from Comma.ai shop if using a White or Gray Panda, Honda Bosch harness for Black Panda
6. USB A-to-C cable
7. USB A-to-A cable or mini USB cable with a Panda Paw from the Comma.ai shop (for flashing White or Gray Pandas from the Pi)
8. A way to power the Pi in your car (battery pack, laptop, or high power 12v power adapter for the car)
9. Minimum 16GB A1-rated microSD card
10. Optional Micro-HDMI adapter plus TV or monitor that accepts HDMI input (the Raspberry Pi 4 does not use the same HDMI connectors as any previous models)
11. Optional USB keyboard
12. A physical Ethernet connection is optional but works best during the install phase
13. Cellular hotspot or home WiFi you can reach from your car (no cable company or retail WiFi)

## MicroSD card preparation and first login

1. Go to https://ubuntu.com/download/raspberry-pi on the laptop that will prepare the microSD card
2. In the Raspberry Pi 4 column, locate the row for Ubuntu 18.04 LTS, and click "Download 64-bit" at the far right end
3. Decompress the `xz`-compressed Ubuntu image file (process varies by Operating System, not required with Balena Etcher)
4. Insert the microSD card into the microSD-to-SD card adapter and then into the computer you will use to burn the image
5. Umount ("eject") the microSD card if the computer automatically mounted it but do not remove it from the slot
6. Burn the decompressed image to the microSD card (process varies by Operating System)
7. When finished, remove and reinsert the SD card adapter. Wait for the Operating System to detect and mount the microSD card.

## Network preparation for the first boot

1. Use your file manager to browse the partition on the card called "system-boot".
2. If you are using Linux to setup the card, run:

`sudo sed -i -e 's/expire: true/expire: false/' user-data`

In MacOS or Windows, open the user-data file for editing and change `expire: true` to `expire: false`.

**Note: The easiest networking option is just plug the Pi into an RJ-45 port on your router or an unused port on a laptop that is using WiFi. Modern Operating Systems make it easy to share your WiFi connection with the Pi this way.**

If you cannot connect your Pi via a physical Ethernet cable, follow these steps to setup WiFi on your Pi prior to first boot.

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
        password: "<enter your hotspot WPA2 password"
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
5. Look for `controlsd`, `boardd`, `ubloxd` and `dashboard` in the rightmost column of the list.
6. If you see all four, let Gernby know you're ready for the model. He will get it to you one way or another.
7. When you receive the model, delete all files in `~/raspilot/models` and unzip the new archive in that folder. The process to copy the file will vary by operating system and selected tool (scp, filezilla, etc.)

**Note: You must ensure that only one file ending with '5' must exist in this folder. If multiple files match, delete or move all but one of the excess files to ~.**

8. Reboot the Pi, and log back in
9. Run `top -u ubuntu` again
10. Look for `controlsd`, `boardd`, `ubloxd`, `dashboard`, and `transcoderd` this time
11. If all five processes are present, you are ready to flash your Panda
12. If you can't hit your household WiFi from your car, be sure to turn on your hotspot before booting up the Pi in the car.

## Flashing the Panda

### Black Panda

1. Connect the Pi to the Black Panda using the standard configuration (USB A on the Panda to USB C on the Pi)
2. Turn on the car
3. SSH into the Pi
4. Edit `~/kegman.json` to add

`"useAutoFlash": "1",`

somewhere in the middle of the file. If you add it to the end, add a trailing comma to the current last line and do not add a trailing comma to this line.

5. Reboot the Pi. The Pi should successfully flash the Panda upon reboot and keep the Panda up to date in the future.

### White or Gray Panda

1. Connect the Pi to the Panda using USB A-to-C. Turn on the car. Wait 2 minutes. Turn the car off.
2. Remove the USB cable from the Panda and move it to an external power source such as a battery pack or laptop USB port
3. Connect the USB A-to-A cable between the Pi and the Panda
4. SSH into the Pi using "ubuntu/ubuntu"
5. Edit `~/kegman.json` to add

`"useAutoFlash": "1",`

somewhere in the middle of the file. If you add it to the end, add a trailing comma to the current last line and do not add a trailing comma to this line.

6. Reboot the Pi using the `sudo reboot` command in ssh. The Panda should successfully flash upon reboot.
7. SSH into the Pi and edit `~/kegman.json` to change

`"useAutoFlash": "1",` to `"useAutoFlash": "0",`

8. This will prevent the Pi from attempting to flash the Panda on subsequent boots as this behavior is not desired for White and Gray Pandas
9. Shutdown the Pi with `sudo halt`
10. Remove the USB A-to-A cable connecting the Pi to the Panda. Move the USB A-to-C cable from the temporary power source back to the Panda.

## First Drive and Training

1. With the Panda flashed and the Pi and Panda connected in the standard configuration, you should be ready to train the software and drive
2. Turn on the car and watch the dash. The `ACC` and `LKAS` indicators should only appear Orange for a couple of seconds before turning Green.
3. After about a minute, the lane marking indicators should light up but show as outlines. You do not need to wait for this to show up before driving.
4. Drive to a road with well-marked lines on both sides of the car and with minimal curves and breaks in the lines -- an interstate is preferred
5. Drive the car on the interstate for 15 minutes. You must hold the wheel as steady as you can and hold the car in the center of the lane as much as possible. This training period is critical to the future performance of the software.
6. After 15 minutes, unplug the Pi from the Panda (either end of the cable is fine), wait a few seconds, and plug it back in. You will see major errors on the dash but they will soon clear up.
7. If the training was successful, you will feel the wheel start to steer by itself after about 75 seconds. **Important:** Note that this will happen even if you haven't set the ACC radar cruise control!
8. Keep your hands lightly brushing the wheel and be ready to take over if at any point you feel uncomfortable about the way the software is steering
9. If you are completely uncomfortable, press the Lane Keeping Assist button beneath the "SET" button in the ACC cruise control section of the steering wheel to disable the Raspberry Pilot LKAS.
10. On all subsequent drives, Raspberry Pilot will start steering the car about 75 seconds after turning on the car. Again, press the LKAS button any time you do not want this feature enabled.

## Notes

The ACC radar cruise control functionality is 100% stock; it remains engaged after a press of the gas and it disengages immediately upon a press of the brakes

Raspberry Pilot defaults to steering 100% of the time, even when the radar cruise (ACC) is not enabled. You may press the Lane Keeping Assist button on the steering wheel to toggle the LKAS functionality on and off at any time. It has a picture of a steering wheel centered between lane lines on a highway.
