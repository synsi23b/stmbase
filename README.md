Quickly initialize a STM project

1. initialize the git repo for the project
2. add and load this as a submodule, dont change the name from stmbase
3. open the submodule in vscode and launch an action, for example "Generate stm32f103c8"
  - this will create and copy some project files into the base folder of the new project
4. open the project with either seger embeded studio (stm32) or IAR workbench (stm8), depending on the chip
  - I prefer to code in vscode and just run the build and debugger with those IDEs thou, therefore, vscode config aslo gets created
5. take a look at *certain synos config file* to change the basic behavior and start from the provided main routine
6. take a look at existing driver packages in stmbase/driver folder. it should be already included to to the main project, just #include what you need

3. replace all occurances of "stm32cubebase" with <yourproject> name in the files
    - .cproject
    - .project
    - stm32cubebase.elf.launch
4. rename stm32cubebase.elf.launch to <yourproject>.elf.launch
5. open the project folder using cubeIDE
6. Build and be happy. Some files contain version specific locations of stuff, be vary


## Configuration for RTOS build and embOSView communication

In your application program, you need to let the compiler know
which build of embOS you are using. This is done by adding the
corresponding define to your preprocessor settings and linking the
appropriate library file.

- OS_LIBMODE_XR    Extremely small release build without Round robin
- OS_LIBMODE_R     Release build
- OS_LIBMODE_S     Release build with stack check
- OS_LIBMODE_SP    Release build with stack check and profiling
- OS_LIBMODE_D     Debug build
- OS_LIBMODE_DP    Debug build with profiling
- OS_LIBMODE_DT    Debug build with trace

# optional

## for synrcp usb serial device communication
1. create a folder called msg in the root directory 
2. create messages with ROS-style msgs and rerun synrpc_gen from its folder whenever you change them
3. there will be sourcefiles added to project for the microcontroller and python files to bridge the gap
4. on linux as admin add a udev rule
  - `echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\" ATTRS{idProduct}==\"5740\", ENV{ID_MM_DEVICE_IGNORE}=\"1\", MODE=\"0666\"" > /etc/udev/rules.d/99-synrpc.rules`
5. install python packages pyusb and libusb backend

# CANopen

## Using CANdlelight adapter on linux master

- install tools
```
sudo add-apt-repository ppa:lely/ppa
agi can-utils liblely-coapp-dev liblely-co-tools python3-dcf-tools
```
- create (or copy from repo) and execute script to bring up the can interface
`setup_candlelight.sh`
```
#!/bin/bash

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```
- test CAN communication using either `candump can0 -L`, with wireshark or with the lely-libraries as a CANopen master. The example was copied from here https://opensource.lely.com/canopen/docs/cmd-tutorial/ It tells node 23 to publish heartbeats every 1500 milliseconds

launch master
```
coctl can0 /etc/coctl.dcf
```
setup network config
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
instruct node 23 to publish heartbeats at 1500ms (write 0x1017 index 0 to 1500)
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
