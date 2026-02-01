# Project: PWM-Based DC Motor Controller with Speed Monitoring via bluetooth device
This project demonstrates how to control a DC motor with an encoder using the STM32F407. It involves a closed-loop control system that uses Pulse Width Modulation (PWM) for speed control, an H-bridge motor driver for direction control, and a timer in encoder mode to read the motor’s actual speed and position.
The main components include:
- **STM32F407VG**
- **GA25-370 DC motor**
- **L298N DC motor driver**
- **JDY-24M Bluetooth module**
  
Using these components, the DC motor can be controlled wirelessly via a Bluetooth connection. The PWM values are displayed on a Bluetooth monitoring application running on a smartphone or laptop.
# Software
The project was programmed using Keil C and is efficient and reliable in operation.
# Demonstration
  ## Breadboard Implementation:
![Breadboard Overview](Motor_controller/DEMO/Img/Messenger_creation_5FF21CC8-20DF-4840-A92C-4F6FC13C26EB.jpeg)

![Breadboard Demonstration](Motor_controller/DEMO/Img/Messenger_creation_C0C6357B-EBC5-4592-BA98-3A49329073EE.jpeg)

  ## PCB Implementation:
![PCB overview](Motor_controller/DEMO/Img/dd23a7bc-59aa-46c7-ae62-9d7ef5148b92.jpg)

![PCB_demonstration](Motor_controller/DEMO/Vid/demo_pcb.mp4)
