# Raspberry Pilot Installation Steps

## Requirements

1. A laptop capable of burning microSD cards and has a decent battery life (min. 30 minutes)
2. Compression software on the above laptop that can decompress the `xz` format (titles and installation process varies by Operating System)
3. Raspberry Pi 4 with 4GB RAM
4. White, Gray, or Black Panda from Comma.ai shop (Note: The Black Panda does not yet work here. We can use some help!)
5. Comma.ai Giraffe or custom-built Giraffe or harness for your car if using a White or Gray Panda
6. USB A-to-C cable
7. USB A-to-A cable or mini USB cable with a Panda Paw from the Comma.ai shop (for flashing the Panda from the Pi)
8. A way to power the Pi in your car (high power 12v to USB adapter, USB port on laptop above, or 12v to 120/240v inverter)
9. Minimum 32GB class 10 microSD card
10. Micro-HDMI adapter (the Raspberry Pi 4 does not use the same HDMI connectors as any previous models)
11. TV or monitor that accepts HDMI input
12. USB keyboard
13. Physical Ethernet connection for the initial installation inside your home
14. Cellular hotspot (or standard WiFi you can reach from your car -- no cable company or retail WiFi)

## MicroSD card preparation and first login

1. Go to https://ubuntu.com/download/raspberry-pi on the laptop that will prepare the microSD card
2. In the Raspberry Pi 4 column, locate the row for Ubuntu 18.04 LTS, and click "Download 64-bit" at the far right end
3. Decompress the `xz`-compressed Ubuntu image file (process varies by Operating System)
4. Insert the microSD card into the microSD-to-SD card adapter and then into the computer you will use to burn the image
5. Umount ("eject") the microSD card if the computer automatically mounted it but do not remove it from the slot
6. Burn the decompressed image to the microSD card (process varies by Operating System)
7. When finished, remove and reinsert the SD card adapter. Wait for the Operating System to detect and mount the microSD card.
8. Use your file manager to browse the partition on the card called "system-boot".
9. In Linux, run `sudo sed -i -e 's/expire: true/expire: false/' user-data` or in other Operating Systems, open the user-data file for editing and change `expire: true` to `expire: false`
10. Open the file called `network-config` for editing.
11. Locate the end of the user instructions with "# Some additional examples are commented out below". Edit the remainder of the file to resemble this example:

```
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional:true
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
12. Save and exit the file.
13. Safely unmount the microSD card. Do not proceed until you know you have safely unmounted.
14. Remove the SD card adapter from the computer and remove the microSD card from the adapter
15. Insert the microSD card into the Rasperry Pi 4 with the contacts facing "up" towards the bottom of the mainboard
16. Connect the keyboard, micro-HDMI adapter and the HDMI cable
17. Connect the USB A end of the USB A-to-C cable to the power supply and the USB C end into the power port on the Pi
18. Wait for the Pi to boot and wait several more seconds after the login is presented to finish the first boot
19. Log into the Pi using "ubuntu" for the username and the password
20. Try to `ping 8.8.8.8`. If successful, continue to the next section. If not, reboot and log in again with ubuntu/ubuntu.

From this point forward, you may continue using the keyboard and monitor to work on the Pi or you may return to your primary computer and log into the Pi using `ssh`.

## Software installation
(Note: You must have stable Internet connectivity at this point to proceed)

1. Log into the Pi using "ubuntu" as the ID and password if you are not still logged in from earlier steps. Clone the repository

`cd ~`  
`git clone https://github.com/Gernby/raspberry-pilot.git`  

2. Ask in Discord for the name of the current branch to checkout.
3. `cd raspberry-pilot`
4. `git checkout <branch name>`
5. `bash start_install.sh`
6. Wait 30 minutes
7. Reboot the Pi and log back in as the "ubuntu" user
8. Run `bash finish_install.sh`
9. Wait about 90 minutes
10. If the process completes successfully, reboot the Pi and log back in as the "ubuntu" user
11. Run the command `top -u ubuntu`
12. Look for `controlsd`, `boardd`, and `dashboard` in the rightmost column of the list.
13. If you see all three, let Gernby know you're ready for the model. He will email it to you.
14. When you receive the model, download the attachment from your email and copy it to `~/raspilot/models` (note that the 's' at the end is critical). The process to copy the file will vary by operating system and selected tool (scp, filezilla, etc.)
15. Reboot the Pi, and log back in
16. Run `top -u ubuntu` again
17. Look for `controlsd`, `boardd`, `dashboard`, and `transcoderd` this time
18. If all four processes are present, you are ready to flash your Panda
19. If you can't hit your household WiFi from your car, edit /etc/netplan/50-cloud-init.yml to comment out the household WiFi defintion before heading out to the car.

## Flashing the Panda
(Note 1: The Pi and the Panda need to be powered separately for this step)

1. If using a White or Gray Panda, separate the Giraffe from the LKAS camera and attach the Comma Power or other continuous power source
2. If using a White or Gray Panda, plug the Panda into the Giraffe
3. Connect the USB A-to-C cable that will power the Pi into the 12-volt power adapter or 120/240-volt AC inverter with high power USB power adapter
4. Turn on your cellular hotspot if you cannot hit your home WiFi from the car
5. Turn on the car if you need to in order to provide power to the Pi
6. Connect the USB A-to-C cable into the power port on the Raspberry Pi
7. Connect the USB A-to-A cable (or mini USB cable for use with the Paw) into the Pi. If you are using the Paw, attach it to the far end of the mini USB cable now.
8. Insert the far end of the USB A-to-A cable or the USB A end of the Comma Paw into the Panda.
(Note: If you are using the Paw, slide the Power switch to 'Off', insert the Paw into the Panda, press and hold the button on the Paw, and while keeping the button pressed, slide the Power switch on the Panda to 'On')
9. On your laptop, ssh into the Pi using the IP address you recorded earlier (either the one from your house WiFi or cellular hotpsot)
10. `cd ~/raspilot`
11. `pipenv run bash flash_panda.sh`
12. Watch the progress and follow any instructions to remove and reseat the USB cable connecting the Pi to the Panda, if prompted
(Note: Watch closely as you may be prompted to move the USB cable from one USB port on the Pi to another within 10 seconds)
13. Take note of the color of the status indicator on the Panda after the flashing process has completed. If the Panda LED is slowly flashing red, the flash was successful. If it is flashing a fast green, it was not successful so please come to Discord to discuss.
14. Shut down the Pi and turn off the car

## Reconfigure and boot up

1. With the Pi and the car powered off, remove all of the USB cables and the Comma Power from the Giraffe
2. Return the Panda and/or Giraffe to the standard configuration (Giraffe and/or Panda are installed between the LKAS camera and the car-side wiring)
3. Use the USB A-to-C cable to connect the USB A port on the Panda with the USB C port on the Pi (Note: DO NOT TRY TO CONNECT THE PI TO A BLACK PANDA USING A USB C-TO-C CABLE. That is the relay box, NOT THE PANDA. The USB C port on the relay box has been electrically altered and will destroy any USB device connected to it, including your Pi.)
3. Start the car. Look for a pulsing red LED if you're using a Gray Panda or a variety of colors on a White Panda.
4. Wait approximately 90 seconds. If the dash goes green and does not flash orange for a split second at the end of the startup process, you should be good to go for a drive.

## Notes

The ACC functionality is 100% stock; it remains engaged after a press of the gas and it disengages immediately upon a press of the brakes

This fork has the option to steer 100% of the time. After your first drive, log in and edit the kegman.json file. Locate the parameter called `lkasMode`. This parameter is set to `0` by default. Leave it set to `0` if you want the car to steer by itself only when you have the ACC (radar cruise) engaged. Change the value to `1` if you want the car to steer by itself at all times, regardless of whether or not the ACC is set.
