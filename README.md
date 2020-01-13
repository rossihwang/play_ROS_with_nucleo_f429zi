# play_ROS_with_nucleo_f429zi

## How

- Using STM32CubeMX and ioc file to generate the MCU code 
- Copy the Inc, Src, startup file, linker file and ioc file into "generated" directory
- Some modifications
  - main.c: remove the main function
  - main.h: declare the functions needed in main.c, remember to remove all the "static"