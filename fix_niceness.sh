#!/bin/bash
sudo renice -n -4 -p `ps -C controlsd -o pid= `
sudo renice -n -3 -p `ps -C boardd -o pid= `
sudo renice -n -2 -p `ps -C transcoderd -o pid= `
sudo renice -n -1 -p `ps -C ubloxd -o pid= `
sudo renice -n 1 -p `ps -C dashboard -o pid= `
sudo taskset -cp --cpu-list 0,1 `ps -C controlsd -o pid= `
sudo taskset -cp --cpu-list 0,1 `ps -C boardd -o pid= `
#sudo taskset -cp --cpu-list 0,1,2 `ps -C ubloxd -o pid= `
sudo taskset -cp --cpu-list 2,3 `ps -C transcoderd -o pid= `
#sudo taskset -cp --cpu-list 0,1,2 `ps -C dashboard -o pid= `
#sudo taskset -cp --cpu-list 0,1,2 `ps -C influxd -o pid= `
sudo chrt -f -a -p 6 `ps -C boardd -o pid= `
sudo chrt -f -a -p 6 `ps -C pandad -o pid= `
sudo chrt -f -a -p 5 `ps -C controlsd -o pid= `
sudo chrt -f -a -p 3 `ps -C ubloxd -o pid= `
sudo chrt -f -a -p 2 `ps -C transcoderd -o pid= `
sudo chrt -f -a -p -1 `ps -C dashboard -o pid= `
cat /sys/class/thermal/thermal_zone0/temp
