1. load this as a Submodule
2. copy the "copy into base" contents into the new project base
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

