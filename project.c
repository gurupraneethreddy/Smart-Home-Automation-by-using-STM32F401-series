#include "stm32f4xx.h"

// Function prototypes
void init_gpio(void);
void init_adc(void);
void configure_pa6_interrupt(void);
void EXTI9_5_IRQHandler(void);
void USART2_Init(void);
char USART2_Read(void);
void USART2_Write(char c);
void USART2_SendString(char *str);

volatile int start_system = 0; // Flag to indicate when the system should start
volatile int adcData; // Make adcData volatile to ensure it's updated in the debugger
const int threshold = 1024;

void delay(int time) {
    for (int i = 0; i < time * 1000; i++);
}

int main(void) {
    init_gpio();               // Initialize GPIOs
    init_adc();                // Initialize ADC
    configure_pa6_interrupt(); // Configure external interrupt on PA6
    USART2_Init();             // Initialize USART2
    
    USART2_SendString("USART Initialized\r\n"); // Initial message for confirmation

    while (!start_system) {
        delay(100); // Small delay in waiting loop
    }

    // Main loop
    while (1) {
        int pir_detected = 0;
        int ultrasonic_detected = 0;
        int ldr_detected = 0;

        // Check PIR Sensor on PA11
        if (GPIOA->IDR & (1U << 11)) { // If motion detected on PA11
            pir_detected = 1;
        }

        // Ultrasonic Sensor: Send trigger pulse
        GPIOA->ODR |= (1U << 4);  // Set PA4 high (Trigger)
        delay(10);                // Short delay ~10 µs
        GPIOA->ODR &= ~(1U << 4); // Set PA4 low (Trigger)

        // Check for echo on PA5
        delay(1); // Delay to wait for the echo response
        if (GPIOA->IDR & (1U << 5)) { // If echo detected on PA5
            ultrasonic_detected = 1;
        }

        // LDR Sensor on PA1 (Analog)
        ADC1->CR2 |= (1 << 30); // Start ADC conversion
        while (!(ADC1->SR & (1 << 1))) {}  // Wait until ADC conversion is complete
        adcData = ADC1->DR;  // Read ADC value

        // Check LDR threshold
        if (adcData <= threshold) {
            ldr_detected = 1;
        }

        // Set or clear the LED on PA8 based on LDR detection
        if (ldr_detected) {
            GPIOA->ODR &= ~(1U << 8); // Turn on LED on PA8
        } else {
            GPIOA->ODR |= (1U << 8);  // Turn off LED on PA8
        }

        // Set or clear the LED on PA10 based on PIR detection
        if (pir_detected) {
            GPIOA->ODR |= (1U << 10); // Turn on LED on PA10
        } else {
            GPIOA->ODR &= ~(1U << 10); // Turn off LED on PA10
        }

        // Set or clear the LED on PA7 based on Ultrasonic detection
        if (ultrasonic_detected) {
            GPIOA->ODR |= (1U << 7); // Turn on LED on PA7
        } else {
            GPIOA->ODR &= ~(1U << 7); // Turn off LED on PA7
        }

        // USART Communication Handling
        if (USART2->SR & (1 << 5)) { // Check if data is received
            char received = USART2_Read(); // Read character from USART

            if (received == '1') {
                GPIOB->ODR |= (1 << 10);   // Turn on the LED (PB10)
                USART2_SendString("LED ON\r\n"); // Send confirmation
            } else if (received == '0') {
                GPIOB->ODR &= ~(1 << 10);  // Turn off the LED (PB10)
                USART2_SendString("LED OFF\r\n"); // Send confirmation
            }
        }
    }
}

void init_gpio(void) {
    RCC->AHB1ENR |= (1U << 0); // Enable GPIOA clock
    RCC->AHB1ENR |= (1U << 1); // Enable GPIOB clock

    // Configure PA0 as input (Push button)
    GPIOA->MODER &= ~(3U << (2 * 0)); // Clear mode bits for PA0
    GPIOA->PUPDR &= ~(3U << (2 * 0)); // Clear pull-up/pull-down bits for PA0
    GPIOA->PUPDR |= (1U << (2 * 0));  // Set pull-up resistor for PA0

    GPIOA->MODER &= ~(3U << (2 * 6)); // Clear mode bits for PA6
    GPIOA->PUPDR &= ~(3U << (2 * 6)); // Clear pull-up/pull-down bits for PA6
    GPIOA->PUPDR |= (1U << (2 * 6));  // Set pull-up resistor for PA6

    // Motion Sensor (PIR) on PA11
    GPIOA->MODER &= ~(3U << (2 * 11)); // Set PA11 as input

    // Ultrasonic Sensor
    GPIOA->MODER |= (1U << (2 * 4));  // Set PA4 as output (Trigger)
    GPIOA->MODER &= ~(3U << (2 * 5)); // Set PA5 as input (Echo)

    // LDR Sensor on PA1
    GPIOA->MODER |= (3U << (2 * 1));  // Set PA1 to analog mode

    // LEDs
    GPIOA->MODER |= (1U << (2 * 8));  // Set PA8 as output for LDR LED
    GPIOA->MODER |= (1U << (2 * 7));  // Set PA7 as output for Ultrasonic LED
    GPIOA->MODER |= (1U << (2 * 10)); // Set PA10 as output for PIR LED
    GPIOB->MODER |= (1U << (2 * 10)); // Set PB10 as output for Bluetooth LED

    // USART2
    GPIOA->MODER &= ~(3 << (2 * 2)); // Clear mode for PA2 (TX)
    GPIOA->MODER |= (2 << (2 * 2));  // Set PA2 to alternate function mode for USART2 TX
    GPIOA->MODER &= ~(3 << (2 * 3)); // Clear mode for PA3 (RX)
    GPIOA->MODER |= (2 << (2 * 3));  // Set PA3 to alternate function mode for USART2 RX

    GPIOA->AFR[0] &= ~(0xF << (2 * 4));  // Clear alternate function for PA2
    GPIOA->AFR[0] |= (7 << (2 * 4));     // Set PA2 to AF7 (USART2)
    GPIOA->AFR[0] &= ~(0xF << (3 * 4));  // Clear alternate function for PA3
    GPIOA->AFR[0] |= (7 << (3 * 4));     // Set PA3 to AF7 (USART2)
}

void init_adc(void) {
    RCC->APB2ENR |= (1 << 8); // Enable ADC1 clock

    ADC1->SMPR2 = 0;          // Set sampling time to 3 cycles for channel 1 (PA1)
    ADC->CCR |= 0;            // Set ADC prescaler to PCLK2/2 without affecting other bits
    ADC1->CR1 = 0;            // Reset CR1 register
    ADC1->CR2 = 0;            // Reset CR2 register
    ADC1->CR2 |= (1U << 1);   // Enable continuous conversion mode
    ADC1->SQR3 = 1;           // Choose channel 1 (PA1) as the first conversion in regular sequence
    ADC1->CR2 |= (1U << 0);   // Enable ADC1
}

void USART2_Init(void) {
    RCC->APB1ENR |= (1 << 17); // Enable USART2 clock
    USART2->BRR = 0x0683;      // Set baud rate to 9600 (assuming 16 MHz clock)
    USART2->CR1 |= (1 << 2);   // Enable RX
    USART2->CR1 |= (1 << 3);   // Enable TX
    USART2->CR1 |= (1 << 13);  // Enable USART2
}

char USART2_Read(void) {
    while (!(USART2->SR & (1 << 5))); // Wait until RXNE (Read data register not empty) is set
    return USART2->DR;
}

void USART2_Write(char c) {
    while (!(USART2->SR & (1 << 7))); // Wait until TXE (Transmit data register empty) is set
    USART2->DR = c;
}

void USART2_SendString(char *str) {
    while (*str) {
        USART2_Write(*str++);
    }
}

void configure_pa6_interrupt(void) {
    RCC->APB2ENR |= (1 << 14);   // Enable SYSCFG clock
    SYSCFG->EXTICR[1] |= 0x0000; // Route PA6 to EXTI6

    EXTI->IMR |= (1 << 6);       // Unmask EXTI6
    EXTI->RTSR |= (1 << 6);      // Trigger interrupt on rising edge
    NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable EXTI9_5 interrupt in NVIC
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1 << 6)) { // Check if interrupt is from PA6
        start_system = 1;      // Set flag to start the system
        EXTI->PR |= (1 << 6);  // Clear pending interrupt flag
    }
}
