# Raspberry Pilot Installation Steps

## Requirements

1. A laptop capable of burning microSD cards and has a decent battery life (min. 30 minutes)
2. Compression software on the above laptop that can decompress the `xz` format (titles and installation process varies by Operating System)
3. Raspberry Pi 4 with 4GB RAM
4. USB A-to-C cable
5. USB A-to-A cable or mini USB cable with a Panda Paw from the Comma.ai shop (for flashing the Panda from the Pi)
6. A way to power the Pi in your car -- either a 12-volt to USB high power adapter or a 12-volt to 120/240-volt inverter with a standard USB power adapter
7. Minimum 32GB class 10 microSD card
8. Micro-HDMI adapter (the Raspberry Pi 4 does not use the same HDMI connectors as any previous models)
9. TV or monitor that accepts HDMI input
10. USB keyboard
11. Physical Ethernet connection for the initial installation inside your home
12. Cellular hotspot (or WiFi you can reach from your car)

## MicroSD card preparation and first login

1. Go to https://ubuntu.com/download/raspberry-pi on the laptop that will prepare the microSD card
2. In the Raspberry Pi 4 column, locate the row for Ubuntu 18.04.4 LTS, and click "Download 64-bit" at the far right end
3. Decompress the `xz`-compressed Ubuntu image file (process varies by Operating System)
4. Insert the microSD card into the microSD-to-SD card adapter and then into the computer you will use to burn the image
5. Umount ("eject") the microSD card if the computer automatically mounted it but do not remove it from the slot
6. Burn the decompressed image to the microSD card (process varies by Operating System)
7. When finished, remove the SD card adapter from the computer and remove the microSD card from the adapter
8. Insert the microSD card into the Rasperry Pi 4. The card inserts with the contacts facing the mainboard.
9. Connect the keyboard, micro-HDMI adapter and the HDMI cable
10. If you are near your Ethernet cable, plug it in. If not, don't worry -- you don't need it yet.
11. Connect the USB cable to household power (not a PC USB port) and then insert the USB C end into the Pi
12. Wait for the Pi to boot and wait several more seconds after the login is presented to finish the first boot
13. Log into the Pi using "ubuntu" for the username and the password
14. You will be forced to change the password. Enter "ubuntu" as the "Current UNIX password" when prompted and then enter your new password twice to change it. From this point forward, you may continue using the keyboard and monitor to work on the Pi or you may return to your primary computer and log into the Pi using `ssh`.

## Software installation
(Note: You must have hardline Ethernet connectivity at this point to proceed)

1. Log into the Pi as the "ubuntu" user using your new password if you are not still logged in from earlier steps. Clone the repository

`cd ~`  
`git clone https://github.com/Gernby/raspberry-pilot.git`  

2. Ask in Discord for the name of the current branch to checkout.
3. `cd raspilot`
4. `git checkout <branch name>`
5. `bash start_install.sh <your WiFi name> <your WiFi password>` (Note that this step adds WiFi support to the Pi and connects to your home WiFi as part of the install)
6. Wait 45 minutes (please verify)
7. Log out of the Pi and log back in as the "ubuntu" user
8. `cd raspilot`
9. `bash finish_install.sh`
10. Wait about 2.5 hours
11. If the process completes successfully, reboot the Pi and log back in as the "ubuntu" user
12. Run the command `top -u ubuntu`
13. Look for `controlsd`, `boardd`, and `laterald` in the rightmost column of the list.
14. If you see all three, let Gernby know you're ready for the model. He will email it to you.
15. When you receive the model, download the attachment from your email and copy it to `~/raspilot/models` (note that the 's' at the end is critical). The process to copy the file will vary by operating system and selected tool (scp, filezilla, etc.)
16. Reboot the Pi, and log back in
17. Run `top -u ubuntu` again
18. Look for `controlsd`, `boardd`, `laterald`, and `transcoderd` this time
19. If all four processes are present, you are ready to flash your Panda
20. If you cannot hit your home WiFi from the car, turn on the hotspot on your phone and connect the Pi to your hotspot WiFi by running the following command:

`nmcli d wifi connect <your hotspot wifi name> password <your hotspot wifi password>`

21. Run `ifconfig` to get the IP address that the Pi was assigned by your home WiFi or cellular hotspot -- whichever one you're planning to use in the car
22. Shut down the Pi (run the `sudo halt` command)

## Flashing the Panda
(Note: The Pi and the Panda need to be powered separately for this step)

1. You need to decide how you plan to power the Pi and Panda separately
2. You may wish to use a separate Giraffe just for flashing this Panda
3. Connect the Comma Power to the OBD-II port if it is not already connected
4. Plug the Panda into the Giraffe you plan to use for flashing (Note that you want a clean CAN bus, so disconnect the Giraffe from the car and the LKAS camera if you only have one Giraffe)
5. Connect the Comma Power into the Giraffe you plan to use for flashing
6. Connect the USB C cable into the 12-volt power adapter or 120/240-volt AC inverter with high power USB power adapter
7. Turn on your cellular hotspot if you cannot hit your home WiFi from the car
8. Turn on the car if you need to in order to provide power to the Pi
9. Connect the USB C cable into the power port on the Raspberry Pi
10. Connect the USB A-to-A cable (or mini USB cable for use with the Paw) into the Pi. If you are using the Paw, attach it to the far end of the mini USB cable now.
11. Insert the far end of the USB A-to-A cable or the USB A end of the Comma Paw into the Panda.
(Note: If you are using the Paw, slide the Power switch to 'Off', insert the Paw into the Panda, press and hold the button on the Paw, and while keeping the button pressed, slide the Power switch on the Panda to 'On')
12. On your laptop, ssh into the Pi using the IP address you recorded earlier (either the one from your house WiFi or cellular hotpsot)
13. `cd ~/raspilot`
14. `pipenv run bash flash_panda.sh`
15. Watch the progress and follow any instructions to remove and reseat the USB cable connecting the Pi to the Panda, if prompted
(Note: Watch closely as you may be prompted to move the USB cable from one USB port on the Pi to another within 10 seconds)
16. Take note of the color of the status indicator on the Panda after the flashing process has completed. If the Panda LED is slowly flashing red, the flash was successful. If it is flashing a fast green, it was not successful so please come to Discord to discuss.
17. Shut down the Pi and turn off the car

## Reconfigure and boot up

1. With the Pi and the car powered off, remove all of the USB cables and the Comma Power from the Giraffe
2. Move the Panda to the Giraffe installed in your car if you used a separate Giraffe just for flashing purposes or reinstall the Giraffe and Panda combination in between the LKAS camera and the car-side wiring bundle if you only have one Giraffe and one Panda
3. Use the USB C cable to connect the USB A port on the Panda with the USB C port on the Pi
3. Start the car. Immediately verify that the Panda is showing a pulsing red LED.
4. Wait approximately 90 seconds. If the dash goes green and does not flash orange for a split second at the end of the startup process, you should be good to go for a drive.

## Notes

The ACC functionality is 100% stock; it remains engaged after a press of the gas and it disengages immediately upon a press of the brakes

However, this fork does attempt to steer 100% of the time by default; it does not wait for you to engage the ACC and the steering does not disengage just because you press the gas or brake. Press the LKAS button on the steering wheel to toggle lateral control on and off.
