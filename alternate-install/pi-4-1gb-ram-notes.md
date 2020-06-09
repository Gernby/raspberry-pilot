# Notes for installing on a Raspberry Pi 4 with 1 GB RAM

This project specifies a Raspberry Pi 4 with 4 GB RAM as the minimum required hardware. Below are some notes for performing the install on a Pi 4 with 1 GB RAM.

1. Complete the first phase of the install (`start-install.sh`), reboot, and log in.
2. Activate a temporary swap file using the following commands:

```
$ touch 3gb
$ dd if=/dev/zero of=3gb bs=1024k count=3072
$ chmod 0600 3gb
$ sudo chown root.root 3gb
$ sudo mkswap 3gb
$ sudo swapon 3gb
```

3. Run `free -h` and look at the last line to ensure the swap space has been activated.
4. Use the `nice` command to lower the CPU priority on the second phase of the build process:

`nice -n 10 sh ./finish-install.sh`

5. Monitor the build. Should take about 2 hours since much of the "RAM" will be in the form of disk space.
6. Reboot after the build completes.
7. Run `free -h` again to ensure the swap space line is showing 0 in all columns.
8. Return to the standard build instructions (i.e. look for the critical processes and get the latest model from Gernby).

That's it. The Pi 4 with 1 GB of RAM should drive the car just fine. It is the build process that is resource intensive.
