#!/bin/bash
sudo renice -n -10 `ps -C controlsd -o pid= `
sudo renice -n -5 `ps -C boardd -o pid= `
sudo renice -n -3 `ps -C transcoderd -o pid= `
sudo renice -n -2 `ps -C ubloxd -o pid= `
sudo renice -n -1 `ps -C dashboard -o pid= `
sudo taskset -cp --cpu-list 0,1,2 `ps -C controlsd -o pid= `
sudo taskset -cp --cpu-list 0,1,2 `ps -C boardd -o pid= `
sudo taskset -cp --cpu-list 0,1,2 `ps -C ubloxd -o pid= `
sudo taskset -cp --cpu-list 3 `ps -C transcoderd -o pid= `
sudo taskset -cp --cpu-list 3 `ps -C dashboard -o pid= `
sudo taskset -cp --cpu-list 3 `ps -C influxd -o pid= `
cat /sys/class/thermal/thermal_zone0/temp
