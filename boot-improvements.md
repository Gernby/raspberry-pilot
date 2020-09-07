 # Raspberry Pi boot improvements
 
 We can improve the boot time of our system with:
 
 ## Remove bootloader delay and raimbow splash screen
 We can add the following to the boot partition ( system-boot) in the config.txt file, after the firmware section.

```
 # Disable the rainbow splash screen
disable_splash=1
# Set the bootloader delay to 0 seconds. The default is 1s if not specified.
boot_delay=0
```
## Overclock the SD card
We can increase the access speed to the SD Card with the following change, add this lines to the usercfg.txt file 

```
# Overclock the SD Card from 50 to 100MHz
# This can only be done with at least a UHS Class 1 card
dtoverlay=sdtweak,overclock_50=100
```

## Disable bluettoth
```
dtoverlay=pi4-disable-bt
```
