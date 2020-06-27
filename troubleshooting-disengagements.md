# Troubleshooting Disengagements

## Description of the Issue

Do you occasionally see error messages flash on your dashboad with Raspberry Pilot, even if you're sitting still? Raspberry Pilot is very sensitive to the microSD card you are using with your Pi 4. For best performance now and in the future, we recommend microSD cards that have earned A2 certification. Those microSD cards provide the performance required to keep the system components running while still meeting the required time intervals when communicating with the car. If the Pi is unable to maintain the frequency of contact with the Panda required to keep the Panda in sync with the car's state, the Panda will relinquish control of the car if only for a couple seconds. This disengagement causes the car to send a "major fault" alert to the dashboard accompanied by warning indicators and a sound. The sound is a simple chime if the radar cruise (ACC) is not controlling the car's speed at the moment but the sound becomes a loud alarm if the disengagement occurs while ACC is in control of the vehicle. It needs to be loud because the car is no longer controlling itself and the driver needs to take control immediately.

The disengagement typically only lasts for a few seconds at a time, but with very low end microSD cards, they can occur quite often -- as much as every 15 to 20 seconds during a drive.

## Description of the Solution

The best way to resolve the disengagements is by using a recommended microSD card with the A2 designation. If you insist on using an A1 card, there are some steps you can take to minimize the less critical tasks that may cause the Pi to miss a heartbeat interval. The Raspberry Pilot install includes the ability to provide very detailed graphical representation of numerous performance characteristics related to lane line detection, angle prediction, and the rate at which it properly navigates a curve. Those charts are still under construction so the system used for generating those charts live does not need to run at the current time. Similarly, the database used to store all of that data also does not need to run currently. The most critical data points regarding overall performance of the system are stored in flat files, so the formal database is not needed. Finally, the process that collects data about the running sytem needs to be told not to try to log performance data to the database since it is no longer running by default.

## Resolution

There are two different versions of the first stage installer we use on the Raspberry Pilot project. The initial version uses traditional commands to compile, install, and configure required packages. The second version uses ansible to standardize the install and customization of various system components. The commands required to shut off these non-critical services will vary based on whether you used the original or the next-generation first stage installer.

If you used the original installer, run these commands:

```
sudo /bin/systemctl disable grafana-server
sudo /bin/systemctl disable influxdb
```

If you used the next-generation installer, run this command that combines both steps:

`ansible localhost -b -m systemd -a "name=influxdb enabled=no,name=grafana-server enabled=no"`

Finally, regardless of the installer you used initially, edit `~/raspilot/controls.sh` to include `-1` as a parameter on the line that launches `dashboard.py`:

`python ~/raspilot/dashboard.py -1 &`

## Conclusion

Making those changes should reduce resource consumption on your Pi enough to eliminate unexpected ACC disengagements and the accompanied dashboard warnings and alert tones.
