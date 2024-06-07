#include "stm32l0xx.h"

// Definiciones de pines para los displays de 7 segmentos y los LEDs del semáforo
#define SEG_A GPIO_PIN_0
#define SEG_B GPIO_PIN_1
#define SEG_C GPIO_PIN_2
#define SEG_D GPIO_PIN_3
#define SEG_E GPIO_PIN_4
#define SEG_F GPIO_PIN_5
#define SEG_G GPIO_PIN_6
#define SEG_DP GPIO_PIN_7

#define GREEN_LED1 GPIO_PIN_8
#define YELLOW_LED1 GPIO_PIN_9
#define RED_LED1 GPIO_PIN_10
#define GREEN_LED2 GPIO_PIN_11
#define YELLOW_LED2 GPIO_PIN_12
#define RED_LED2 GPIO_PIN_13

// Pines del teclado
const uint16_t rowPins[4] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11}; // Filas del teclado
const uint16_t colPins[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15}; // Columnas del teclado

// Mapa de los dígitos en el display de 7 segmentos
const uint8_t digitSegments[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Duración de las señales del semáforo en milisegundos
unsigned long greenDuration = 5000;
unsigned long yellowDuration = 2000;
unsigned long redDuration = 5000;

// Variables para el control del tiempo
volatile unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long timeRemaining = greenDuration;
int currentNumber = 0;

typedef enum { WAITING, GREEN, YELLOW, RED, CONFIG_GREEN, CONFIG_YELLOW, CONFIG_RED } TrafficLightState;
TrafficLightState state = WAITING;

char inputBuffer[4]; // Buffer para entrada de tiempo
int inputIndex = 0;

void SystemClock_Config(void);
void setup(void);
void loop(void);
void displayNumber(int display, int number);
void displayDoubleNumber(int number);
int bufferToInt(void);
void updateTrafficLights(TrafficLightState state);
char getKey(void);
void delay_ms(unsigned long ms);
void TIM2_IRQHandler(void);

void SystemClock_Config(void) {
    // Configurar el reloj del sistema
    RCC->CR |= RCC_CR_HSION; // Habilitar HSI
    while ((RCC->CR & RCC_CR_HSIRDY) == 0); // Esperar a que HSI esté listo
    RCC->CFGR |= RCC_CFGR_SW_HSI; // Seleccionar HSI como fuente de reloj del sistema
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Esperar a que HSI sea usado
}

void setup(void) {
    // Habilitar el reloj GPIO
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Configurar pines de los segmentos del display como salida
    for (int i = 0; i < 8; i++) {
        GPIOA->MODER |= (1 << (i * 2)); // PA0 - PA7 como salida
        GPIOB->MODER |= (1 << (i * 2)); // PB0 - PB7 como salida
    }

    // Configurar pines de los LEDs como salida
    GPIOA->MODER |= (GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0 | 
                     GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0);

    // Configurar pines del teclado numérico como entrada con pull-down
    for (int i = 0; i < 4; i++) {
        GPIOB->MODER &= ~(3 << (rowPins[i] * 2));
        GPIOB->PUPDR |= (2 << (rowPins[i] * 2));
    }

    for (int i = 0; i < 4; i++) {
        GPIOB->MODER &= ~(3 << (colPins[i] * 2));
        GPIOB->PUPDR |= (2 << (colPins[i] * 2));
    }

    // Configurar el temporizador TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Habilitar el reloj para TIM2
    TIM2->PSC = 15999; // Prescaler: 16 MHz / (15999 + 1) = 1 kHz
    TIM2->ARR = 999; // Auto-reload: 1000 ms / 1 kHz = 1 segundo
    TIM2->DIER |= TIM_DIER_UIE; // Habilitar la interrupción de actualización
    TIM2->CR1 |= TIM_CR1_CEN; // Habilitar el contador

    NVIC_EnableIRQ(TIM2_IRQn); // Habilitar la interrupción TIM2
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Verificar el flag de actualización
        TIM2->SR &= ~TIM_SR_UIF; // Limpiar el flag de actualización
        currentMillis++; // Incrementar el contador de milisegundos
    }
}

void loop(void) {
    char key = getKey();

    if (key != 0) {
        if (state == WAITING) {
            if (key == 'A') {
                state = GREEN;
                previousMillis = currentMillis;
                timeRemaining = greenDuration / 1000;
            } else if (key == 'B') {
                state = CONFIG_GREEN;
                inputIndex = 0;
                displayDoubleNumber(greenDuration / 1000); // Mostrar el tiempo actual del LED verde
            } else if (key == 'C') {
                state = CONFIG_YELLOW;
                inputIndex = 0;
                displayDoubleNumber(yellowDuration / 1000); // Mostrar el tiempo actual del LED amarillo
            } else if (key == 'D') {
                state = CONFIG_RED;
                inputIndex = 0;
                displayDoubleNumber(redDuration / 1000); // Mostrar el tiempo actual del LED rojo
            }
        } else if (state == GREEN || state == YELLOW || state == RED) {
            if (key == 'A') {
                state = WAITING;
            } else if (key == 'B') {
                state = CONFIG_GREEN;
                inputIndex = 0;
                displayDoubleNumber(greenDuration / 1000); // Mostrar el tiempo actual del LED verde
            } else if (key == 'C') {
                state = CONFIG_YELLOW;
                inputIndex = 0;
                displayDoubleNumber(yellowDuration / 1000); // Mostrar el tiempo actual del LED amarillo
            } else if (key == 'D') {
                state = CONFIG_RED;
                inputIndex = 0;
                displayDoubleNumber(redDuration / 1000); // Mostrar el tiempo actual del LED rojo
            }
        } else if (state == CONFIG_GREEN) {
            if (key >= '0' && key <= '9' && inputIndex < 3) {
                inputBuffer[inputIndex++] = key;
                displayDoubleNumber(bufferToInt());
            } else if (key == '#') {
                greenDuration = bufferToInt() * 1000;
                inputIndex = 0;
                memset(inputBuffer, 0, sizeof(inputBuffer)); // Limpiar el buffer de entrada
                state = WAITING;
            }
        } else if (state == CONFIG_YELLOW) {
            if (key >= '0' && key <= '9' && inputIndex < 3) {
                inputBuffer[inputIndex++] = key;
                displayDoubleNumber(bufferToInt());
            } else if (key == '#') {
                yellowDuration = bufferToInt() * 1000;
                inputIndex = 0;
                memset(inputBuffer, 0, sizeof(inputBuffer)); // Limpiar el buffer de entrada
                state = WAITING;
            }
        } else if (state == CONFIG_RED) {
            if (key >= '0' && key <= '9' && inputIndex < 3) {
                inputBuffer[inputIndex++] = key;
                displayDoubleNumber(bufferToInt());
            } else if (key == '#') {
                redDuration = bufferToInt() * 1000;
                inputIndex = 0;
                memset(inputBuffer, 0, sizeof(inputBuffer)); // Limpiar el buffer de entrada
                state = WAITING;
            }
        }
    }

    // Actualizar estado del semáforo basado en el tiempo transcurrido
    if (state == GREEN || state == YELLOW || state == RED) {
        switch (state) {
            case GREEN:
                if (currentMillis - previousMillis >= greenDuration / 1000) {
                    state = YELLOW;
                    previousMillis = currentMillis;
                    timeRemaining = yellowDuration / 1000;
                } else {
                    timeRemaining = (greenDuration / 1000 - (currentMillis - previousMillis));
                }
                break;
            case YELLOW:
                if (currentMillis - previousMillis >= yellowDuration / 1000) {
                    state = RED;
                    previousMillis = currentMillis;
                    timeRemaining = redDuration / 1000;
                } else {
                    timeRemaining = (yellowDuration / 1000 - (currentMillis - previousMillis));
                }
                break;
            case RED:
                if (currentMillis - previousMillis >= redDuration / 1000) {
                    state = GREEN;
                    previousMillis = currentMillis;
                    timeRemaining = greenDuration / 1000;
                } else {
                    timeRemaining = (redDuration / 1000 - (currentMillis - previousMillis));
                }
                break;
        }

        // Mostrar el tiempo restante en el display 1
        displayDoubleNumber(timeRemaining);
    } else if (state == WAITING) {
        displayDoubleNumber(0); // Mostrar 00 en el display cuando está esperando
    }
    
    updateTrafficLights(state);
}

void displayNumber(int display, int number) {
    uint8_t segments = digitSegments[number];
    for (int i = 0; i < 8; i++) {
        if (segments & (1 << i)) {
            if (display == 1) {
                GPIOA->ODR |= (1 << i);
            } else {
                GPIOB->ODR |= (1 << i);
            }
        } else {
            if (display == 1) {
                GPIOA->ODR &= ~(1 << i);
            } else {
                GPIOB->ODR &= ~(1 << i);
            }
        }
    }
}

void displayDoubleNumber(int number) {
    int tens = number / 10;
    int ones = number % 10;
    displayNumber(1, tens);
    displayNumber(2, ones);
}

int bufferToInt(void) {
    int number = 0;
    for (int i = 0; i < inputIndex; i++) {
        number = number * 10 + (inputBuffer[i] - '0');
    }
    return number;
}

void updateTrafficLights(TrafficLightState state) {
    switch (state) {
        case GREEN:
            GPIOA->ODR |= GREEN_LED1;
            GPIOA->ODR &= ~YELLOW_LED1;
            GPIOA->ODR &= ~RED_LED1;
            GPIOA->ODR &= ~GREEN_LED2;
            GPIOA->ODR &= ~YELLOW_LED2;
            GPIOA->ODR |= RED_LED2;
            break;
        case YELLOW:
            GPIOA->ODR &= ~GREEN_LED1;
            GPIOA->ODR |= YELLOW_LED1;
            GPIOA->ODR &= ~RED_LED1;
            GPIOA->ODR &= ~GREEN_LED2;
            GPIOA->ODR &= ~YELLOW_LED2;
            GPIOA->ODR |= RED_LED2;
            break;
        case RED:
            GPIOA->ODR &= ~GREEN_LED1;
            GPIOA->ODR &= ~YELLOW_LED1;
            GPIOA->ODR |= RED_LED1;
            GPIOA->ODR |= GREEN_LED2;
            GPIOA->ODR &= ~YELLOW_LED2;
            GPIOA->ODR &= ~RED_LED2;
            break;
        case WAITING:
        case CONFIG_GREEN:
        case CONFIG_YELLOW:
        case CONFIG_RED:
            GPIOA->ODR &= ~GREEN_LED1;
            GPIOA->ODR &= ~YELLOW_LED1;
            GPIOA->ODR &= ~RED_LED1;
            GPIOA->ODR &= ~GREEN_LED2;
            GPIOA->ODR &= ~YELLOW_LED2;
            GPIOA->ODR &= ~RED_LED2;
            break;
        default:
            break;
    }
}

char getKey(void) {
    for (int row = 0; row < 4; row++) {
        // Configurar la fila como salida
        GPIOB->MODER |= (1 << (rowPins[row] * 2));
        GPIOB->ODR |= (1 << rowPins[row]);

        for (int col = 0; col < 4; col++) {
            if (GPIOB->IDR & (1 << colPins[col])) {
                // Configurar la fila como entrada
                GPIOB->MODER &= ~(3 << (rowPins[row] * 2));
                return keys[row][col];
            }
        }

        // Configurar la fila como entrada
        GPIOB->MODER &= ~(3 << (rowPins[row] * 2));
    }

    return 0;
}

int main(void) {
    SystemClock_Config();
    setup();

    while (1) {
        loop();
    }
}
