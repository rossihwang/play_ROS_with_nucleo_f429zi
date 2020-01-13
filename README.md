# play_ROS_with_nucleo_f429zi

## How the MCU code generated

- Using STM32CubeMX and ioc file to generate the MCU code 

- Copy the Inc, Src, startup file, linker file and ioc file into "generated" directory

- Some modifications

  - main.c: remove the main function
  - main.h: declare the functions needed in main.c, remember to remove all the "static"

- Notion that the "STM32 Firmware" is treated as external source 

  

## How to use the rosserial_client example

- Compile the project and flash the bin file

- In ROS

  ```
  rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/ACM0
  ```

  ```
  rostopic echo chatter1  // or chatter2
  ```

  

