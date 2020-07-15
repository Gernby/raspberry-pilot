#!/bin/bash
sudo renice -n -1 `ps -C boardd -o pid= `
sudo renice -n -1 `ps -C controlsd -o pid= `
sudo renice -n -1 `ps -C dashboard -o pid= `
sudo renice -n -1 `ps -C transcoderd -o pid= `
sudo taskset -cp --cpu-list 0,1 `ps -C controlsd -o pid= `
sudo taskset -cp --cpu-list 0,1 `ps -C boardd -o pid= `
sudo taskset -cp --cpu-list 2,3 `ps -C transcoderd -o pid= `
sudo taskset -cp --cpu-list 2,3 `ps -C dashboard -o pid= `
sudo taskset -cp --cpu-list 2,3 `ps -C influxd -o pid= `
cat /sys/class/thermal/thermal_zone0/temp
