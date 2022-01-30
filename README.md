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

