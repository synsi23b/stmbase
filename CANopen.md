# CANopen

## Using CANdlelight adapter on linux master

- install tools (Lely bridge)

https://opensource.lely.com/canopen/docs/installation/

```
sudo add-apt-repository ppa:lely/ppa
sudo apt install can-utils liblely-coapp-dev liblely-co-tools python3-dcf-tools
```
- create (or copy from repo) and execute script to bring up the can interface `setup_candlelight.sh`
```
#!/bin/bash

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```
- test CAN communication using either `candump can0 -L`, with wireshark or with the lely-libraries as a CANopen master. The example was copied from here https://opensource.lely.com/canopen/docs/cmd-tutorial/ 

## Use CANopen control tool from Lely Bridge

https://opensource.lely.com/canopen/docs/coctl/

### setup CANdlelight adapter

if you created the executable script from the first step run `./setup_candlelight.sh`

## start the CANopen control tool and configure a new node id

```
sudo coctl can0 /etc/coctl.dcf
[1] set network 1
[1] OK
[2] set id 1
[2] OK
[3] init 2
coctl: NMT: entering reset application state
coctl: NMT: entering reset communication state
coctl: NMT: running as master
coctl: NMT: entering pre-operational state
coctl: NMT: entering operational state
[3] OK
```
- scan for uncofigured lit-stepper boards (flickering red LED)
```
[4] _lss_fastscan 0x6c656170 0xFFFFFFFF 0x73746570 0xFFFFFFFF 1 0xFFFFFFFF 0 0
[4] 0x6c656170 0x73746570 0x00000001 0x81122559
```
- In the example, the board with serial number 0x81122559 answered our call and its green LED started flickering.
- give the board a valid, unique node id on the bus (2 .. 127). in the example, we give it id 23
```
[5] lss_set_node 23
[5] OK
[6] lss_store
[6] OK
```
- now, on reboot the device will start with id 23

## setting another id for a board, that was already configured

we are going to change the id of the board, which was previously configured with the ID 24 and give it the new id 64

- start coctl-master as in the example before
- to change a configured board, we need to know the serial number
```
[55] 24 read 0x1018 4 u32
[55] 0x81122559
```
- with the serialnumber `0x81122559` read from the device, initiate hte LSS session
```
[56] lss_switch_sel 0x6c656170 0x73746570 1 0x81122559
[56] OK
```
- the green led should be flickering, now set the new node id, store and restart the device for the change to take effect
```
[57] lss_set_node 64
[57] OK
[58] lss_store
[58] OK
```

### launch master / cmd line tool
```
sudo coctl can0 /etc/coctl.dcf
```

### setup network config
```
> [1] set network 1     # Set the default network-ID, so it can be omitted.
< [1] OK
> [2] set id 1          # Set the node-ID of this device to 1.
< [2] OK
> [3] init 2            # Initialize the device, and configure the CAN bus with
> [3]                   # with a bit rate of 500 kbit/s. The number 2 is the
                        # value for 500kbit/s. 0 as param is 1000kbit/s
< coctl: error: unable to set bitrate of vcan0 to 500000 bit/s
< coctl: NMT: entering reset application state
< coctl: NMT: entering reset communication state
< coctl: NMT: running as master
< coctl: NMT: entering pre-operational state
< coctl: NMT: entering operational state
< [3] OK
```

### instruct node 23 to publish heartbeats at 1500ms (write 0x1017 index 0 to 1500)
```
[4] 23 write 0x1017 0 u16 1500
1 23 BOOT_UP
1 23 USER BOOT A (The CANopen device is not listed in object 1F81.)
1 21 BOOT_UP
1 21 USER BOOT A (The CANopen device is not listed in object 1F81.)
[4] OK
```

### Decode protocol with wireshark

- run wireshark and listen to can0
- set the protocol `Analyze > Decode As... > Field:Can next level dissector, Current:canOpen`
