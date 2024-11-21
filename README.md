# Smart-Home-Automation-by-using-STM32F401-series

This code is an STM32 microcontroller program that initializes and configures various GPIO pins, an ADC, and a USART to monitor sensors (PIR, ultrasonic, LDR) and communicate with an external device via Bluetooth. The system starts only when PA6 is pressed, triggering an external interrupt.

Key Components:

GPIO Configuration: Configures pins for sensors and LEDs.
ADC Initialization: Configures an ADC channel to read data from an LDR sensor.
USART Initialization: Sets up serial communication over USART2 for data exchange.
Interrupt Handling: An external interrupt on PA6 is configured to start the system.
Sensor Reading and LED Control: The main loop reads sensor values and controls LEDs based on conditions.
Functionality:

The program waits for a signal on PA6 (configured with an interrupt) to start.
Once started, it continuously monitors:
PIR Sensor on PA11: Detects motion, turning on PA10 LED when triggered.
Ultrasonic Sensor: Sends a trigger pulse on PA4 and checks for an echo on PA5 to detect presence, controlling the PA7 LED.
LDR Sensor on PA1: Checks light levels using the ADC and controls PA8 LED based on a threshold.
The program also communicates over USART2 to control a Bluetooth LED on PB10 based on commands ('1' to turn on, '0' to turn off).


# STM32 Sensor Monitoring and Home Control System
This project is a microcontroller program for the STM32F4 series that reads sensor data (PIR, ultrasonic, LDR), controls LEDs based on sensor states, and communicates over USART2. The system initializes upon pressing PA6, which triggers an external interrupt to start sensor monitoring.

Components and Setup
PA6: Push-button to start the system (interrupt-based).
PA11: PIR sensor for motion detection.
PA4/PA5: Ultrasonic sensor with trigger (PA4) and echo (PA5).
PA1: LDR sensor using ADC for light intensity.
PA8, PA7, PA10: LEDs for LDR, Ultrasonic, PIR sensors.
PB10: LED controlled over Bluetooth commands.
USART2 (PA2 - TX, PA3 - RX): Serial communication for external commands.


Project Setup:

Open Keil 5, create a new project, and select the STM32F4xx series as your device.
Go to Manage Run-Time Environment and enable Device > STM32Cube HAL and CMSIS for STM32F4.
In Device > Startup, enable Startup and add system_stm32f4xx.c to your project.
Add Source Files:

Add your source code file (.c) to the project in Project Explorer.
Include STM32 headers (e.g., stm32f4xx.h, system_stm32f4xx.h) and configure any custom libraries for USART, GPIO, or ADC.
Configure USART:

In Configure Flash settings, go to USART2 and set the baud rate to 9600.
Ensure PA2 and PA3 are set for alternate function mode in the GPIO configuration.
ADC and Interrupt Setup:

In the GPIO setup, configure PA1 for analog input.
Enable the interrupt for EXTI9_5 in the Manage Run-Time Environment > CMSIS > RTOS2.
Build and Flash:

Compile the project, fix any build errors, and flash the code onto the STM32.
Usage
Connect the STM32 board with sensors and LEDs as per the schematic.
Use PA6 to start the system. The microcontroller will read sensor inputs and control LEDs accordingly.
Use USART to send '1' or '0' commands to control PB10 LED.




[![Smart-Home-Automation-by-using-STM32F401-series](https://img.youtube.com/vi/YpoFAt6k4QQ)](https://www.youtube.com/watch?v=YpoFAt6k4QQ)
