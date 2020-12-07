#!/bin/bash
sudo renice -n -19 -p `ps -C boardd -o pid= `
sudo renice -n -19 -p `ps -C controlsd -o pid= `
sudo renice -n -5 -p `ps -C transcoderd -o pid= `
sudo renice -n -1 -p `ps -C ubloxd -o pid= `
sudo renice -n -1 -p `ps -C dashboard -o pid= `
sudo taskset -cp -a --cpu-list 4,5 `ps -C controlsd -o pid= `
sudo taskset -cp -a --cpu-list 4,5 `ps -C boardd -o pid= `
sudo taskset -cp -a --cpu-list 4,5 `ps -C ubloxd -o pid= `
#sudo taskset -cp --cpu-list 2,3 `ps -C transcoderd -o pid= `
sudo taskset -cp -a --cpu-list 0,1 `ps -C dashboard -o pid= `
sudo taskset -cp -a --cpu-list 0,1 `ps -C influxd -o pid= `
sudo chrt -f -a -p 19 `ps -C boardd -o pid= `
sudo chrt -f -a -p 19 `ps -C pandad -o pid= `
sudo chrt -f -a -p 19 `ps -C controlsd -o pid= `
sudo chrt -f -a -p 1 `ps -C ubloxd -o pid= `
sudo chrt -f -a -p 5 `ps -C transcoderd -o pid= `
sudo chrt -f -a -p 1 `ps -C dashboard -o pid= `
cat /sys/class/thermal/thermal_zone0/temp
