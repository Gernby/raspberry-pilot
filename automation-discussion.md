# Automating Tasks with a Mobile App

## Background

A big part of testing Raspberry Pilot is a process commonly referred to as "A/B testing". We are constantly being asked to report which of multiple settings and driving models are better or worse than others. It helps to be able to quickly and easily update settings in configuration files or replacing complete files and rebooting. Since the Raspberry Pi is running Linux, the image includes a running SSH server. SSH allows users to execute commands on remote hosts, using the ssh command as an authentication mechanism. By crafting specific command lines or small scripts, it becomes easy to make critical changes to your copy of Raspberry Pilot remotely. Finally, when you add an SSH app to your phone that supports remote command execution, now you can change settings or replace driving models in Raspberry Pilot just by pressing buttons on your phone screen.

See Raspberry SSH as one such example app on Android, available here: https://play.google.com/store/apps/details?id=uk.co.knowles_online.raspberryssh&hl=en_US There is a Lite version available, but only the paid version allows you to create enough automation buttons to meet our needs.

## Configuring WiFi

You may have multiple Raspberry Pi 4s configured with Raspberry Pilot. In order to use the same app to remote control each Pi, you're going to need to configure at least one Pi to use the same IP address as the one (or more) other Pis you are using. Otherwise, any work you put into establishing your remote control testing suite will only work on one Pi. Here are the commands for configuring that:

nmcli d wifi connect <your hotspot SSID> password <your hotspot WiFi password>
sudo nmcli con edit <your hotspot SSID>

Then, in the "nmcli interactive connection editor", run

nmcli> set ipv4.address 192.168.43.89
Do you also want to set 'ipv4.method' to 'manual'? [yes]: y
nmcli> save
Connection '<your hotspot SSID' (9d08f0e6-8e64-4947-b54b-7ab5cc4ced64) successfully updated.

## Switching models

The first approach to automating tasks covered here includes specifying commands as part of the SSH session establishment. This is useful for small, simple tasks such as removing the current hdf5 files from ~/raspilot/models and copying a different one into the folder. I created a folder structure for holding each of the hdf5 files being tested under `/home/ubuntu/buttons/model-*`, with `model-*` meaning "model-1", "model-2", "model-3", and "model-4". I placed one hdf5 file in each folder and will replace those files when new model bundles are released. Then I created buttons in the mobile app that erase the current model from ~/raspilot/models, copies a specific hdf5 file over to the models folder, and reboots the Raspberry Pi to make the change take effect. For example, this is the command behind the "Model 1" button in my Raspberry SSH mobile app:

`/bin/touch /home/ubuntu/models/5 && /bin/rm /home/ubuntu/raspilot/models/*5 && /bin/cp /home/ubuntu/buttons/model-1/* /home/ubuntu/raspilot/models && /usr/bin/sudo /sbin/reboot`

I have four Model buttons, each one corresponding to loading an hdf5 file sitting in a specific directory before rebooting the Pi to make it take effect. When new models are published, I'll copy the new scalers into the models folder and place up to four hdf5 files in each of the model folders associated with the buttons.

## Changing values in kegman.json

The second approach is a little more complex and must be performed in multiple steps. First, the user clicks a button to choose which parameter they wish to modify. That will store the parameter name in a file called `param`. Next, they will indicate whether the value is going to be increased or decreased. This is stored as either a literal + or - in a file called `plus-minus`. Finally, the user will indicate how much to change the value of that parameter. That amount is stored in a file called `delta`. Once all three selections have been made, the user will then click a button called `Submit change` which calls a backend script, located here:

https://github.com/Gernby/raspberry-pilot/blob/gh-pages/changevalue.sh

The script will read the three files that were created and populated by the first three user actions. It will find the current value stored in kegman.json, perform some string manipulation and some basic math to derive a new value, then replace the value of the parameter in a temporary copy of kegman.json. Then, the script backs up the current version of kegman.json before overwriting it with the new version just updated. The script then deletes all temporary files used during the update to prevent accidentally reusing any values left over from prior changes.

## Recommended values for key parameters

When deciding which parameters to change and which values to try for those parameters, see these recommended settings and their typical range:

```
dampTime: 0.0 to 0.25
dampMPC: 0.0 to 0.2
reactMPC: 0.0 to 0.3
polyFactor: 0.0 to 0.5
polyDamp: 0.0 to 0.5
```
## Future plans

See `changevalue.sh` for a working Proof of Concept of the backend script written in bash. A final version is currently being developed in Python. Additionally, there are millions of combinations of settings in kegman.json. If we develop a way to automatically adjust certain settings in kegman.json once every minute (for example), I'm going to need a button that stops the automatic updates if the system lands on a good setting.
