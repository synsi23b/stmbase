#!/bin/bash

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0