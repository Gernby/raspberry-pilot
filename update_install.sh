#!/bin/bash

(sudo crontab -l; echo "@reboot sleep 60; bash /home/ubuntu/raspilot/fix_niceness.sh";) | sudo crontab -
sudo crontab -l

sudo cp ~/raspilot/phonelibs/influxdb.conf /etc/influxdb/influxdb.conf

sudo chown -R 1000:1000 ~/raspilot/node-red
sudo docker run -it --network host -v /home/ubuntu/raspilot/node-red:/data --name nodered-raspilot nodered/node-red 
