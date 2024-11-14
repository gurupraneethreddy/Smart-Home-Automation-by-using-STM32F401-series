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
