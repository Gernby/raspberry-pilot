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

1. Please come to the #general channel in Discord to request the current image
2. Download the latest image on the laptop that will prepare the microSD card
3. Insert the microSD card into the microSD-to-SD card adapter and then into the computer you will use to burn the image
4. If you have Balena Etcher, flash the SD card with the image you just downloaded
5. If you are not using Balena Etcher, decompress the `xz`-compressed Ubuntu image file
6. Umount ("eject") the microSD card if the computer automatically mounted it but do not remove it from the slot
7. Burn the decompressed image to the microSD card (process varies by Operating System)

## First boot

1. Safely unmount the microSD card. Do not proceed until you know you have safely unmounted.
2. Remove the SD card adapter from the computer and remove the microSD card from the adapter
3. Insert the microSD card into the Rasperry Pi 4 with the contacts facing "up" towards the bottom of the mainboard
4. If you are using a local console, connect the keyboard, micro-HDMI adapter and the HDMI cable
5. If you are not using a local console, connect the Pi to a physical Ethernet connection using an Ethernet cable. This can be done either by connecting the Pi directly to your router or to a laptop with an unused RJ-45 port if you are familiar with the process.
6. Connect the Pi to a high-power USB port via the USB A-to-C cable.
7. Allow the Pi to boot and wait for at least one minute. If you are using the console, wait for a few lines of text to appear before attempting to log in.
8. Locate the Pi IP address in your home router or your laptop if you are using your laptop to provide a network connection to the Pi
9. Log into the Pi using "ubuntu" for the username and "raspilot" for the password
10. Try to `ping -c 5 8.8.8.8`. If successful, continue to the next section. If not, reboot and log in again with ubuntu/ubuntu.
11. Use the `nmcli` command to configure all of the WiFi access points you want to use in the car (home WiFi, cellular hotsopt). Run this command only once for every WiFi network name you plan to use:

`sudo nmcli d wifi connect <SSID> password <password>`

## Flashing the Panda

**Note: If you can't hit your household WiFi from your car, be sure to configure the Pi to hit your cellular hotspot and turn on your hotspot before booting up the Pi in the car.**

Bring the Pi out the car. If you're using a White or Gray Panda, be sure to also bring a separate power supply and USB A to A cable.

1. For the Black Panda, connect the Pi to the Black Panda using the standard configuration (USB A on the Panda to USB C on the Pi)
2. For the White or Gray Panda, connect the power supply to the Pi via the USB C port and connect the Pi to the Panda using the USB A to A cable
3. Turn on the car
4. SSH into the Pi
5. Run `sudo sudo sh ~/raspilot/flash_panda.sh recover`
6. Once the flash is successful, you may wish to reboot the Pi before going out for your first drive.

If you're using a White or Gray Panda, remember to unplug the power supply from the Pi and remove the USB A-to-A cable from the Pi and the Panda. Then connect the Pi to the Panda using the standard configuration (USB A in the Panda to USB C on the Pi). The USB A to A cable is not used under normal conditions.

If you have successfully flashed the Panda, you are ready to calibrate and go for your first drive. If you are unable to flash the Panda or are not convinced that you have, come to Discord to discuss the issue. 

## First Drive and Training

1. With the Panda flashed and the Pi and Panda connected in the standard configuration, you should be ready to train the software and drive
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
