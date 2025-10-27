#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/lm4f/systemcontrol.h>
#include <libopencm3/lm4f/rcc.h>
#include <libopencm3/lm4f/gpio.h>
#include <libopencm3/lm4f/uart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Pin definitions for EK-TM4C1294XL LaunchPad
// LED pins
#define LED_RED_PORT    GPIOF
#define LED_RED_PIN     GPIO0    // PF0 - Red LED (D1)

#define LED_GREEN_PORT  GPIOF
#define LED_GREEN_PIN   GPIO4    // PF4 - Green LED (D2)

#define LED_BLUE_PORT   GPIOF
#define LED_BLUE_PIN    GPIO1    // PF1 - Blue LED (D3)

// Button pin
#define BUTTON_PORT     GPIOJ
#define BUTTON_PIN      GPIO0    // PJ0 - SW1

// Global state
SemaphoreHandle_t xLedMutex;
volatile bool redLedState = false;

// Function prototypes
static void clock_setup(void);
static void gpio_setup(void);
static void uart_setup(void);
void uart_puts(const char *s);

// Task declarations
void TaskHeartbeat(void *pvParameters);
void TaskButtonMonitor(void *pvParameters);
void TaskLedControl(void *pvParameters);

static void clock_setup(void) {
    // Configure for 120MHz using PLL
    rcc_sysclk_config(OSCSRC_MOSC, XTAL_25M, 4);  // 25MHz / 5 * 24 / 1 = 120MHz
}

static void gpio_setup(void) {
    // Enable GPIO ports
    periph_clock_enable(RCC_GPIOF);  // LEDs
    periph_clock_enable(RCC_GPIOJ);  // Button

    // Configure LED pins as outputs
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GREEN_PIN);
    gpio_mode_setup(LED_BLUE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_BLUE_PIN);

    // Configure button pin as input with pull-up
    gpio_mode_setup(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BUTTON_PIN);

    // Turn all LEDs off initially
    gpio_clear(LED_RED_PORT, LED_RED_PIN);
    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
    gpio_clear(LED_BLUE_PORT, LED_BLUE_PIN);
}

static void uart_setup(void) {
    // Enable UART0 clock
    periph_clock_enable(RCC_GPIOA);
    periph_clock_enable(RCC_UART0);

    // Setup GPIO pins for UART0 (PA0=RX, PA1=TX)
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_set_af(GPIOA, 1, GPIO0 | GPIO1);  // AF1 for UART0

    // Setup UART parameters: 115200 8N1
    uart_set_baudrate(UART0, 115200);
    uart_set_databits(UART0, 8);
    uart_set_stopbits(UART0, 1);
    uart_set_parity(UART0, UART_PARITY_NONE);

    // Enable UART
    uart_enable(UART0);
}

void uart_puts(const char *s) {
    while (*s) {
        uart_send_blocking(UART0, *s++);
    }
}

// Task 1: Heartbeat - blink blue LED every 1 second
void TaskHeartbeat(void *pvParameters) {
    (void) pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    bool state = false;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        state = !state;
        if (state) {
            gpio_set(LED_BLUE_PORT, LED_BLUE_PIN);
        } else {
            gpio_clear(LED_BLUE_PORT, LED_BLUE_PIN);
        }
    }
}

// Task 2: Button monitor - check for button presses
void TaskButtonMonitor(void *pvParameters) {
    (void) pvParameters;

    bool lastButtonState = true;  // Pull-up, so HIGH when not pressed

    for (;;) {
        // Read button state (active low)
        uint16_t port_val = gpio_port_read(BUTTON_PORT);
        bool currentButtonState = (port_val & BUTTON_PIN) ? true : false;

        // Detect button press (transition from HIGH to LOW)
        if (!currentButtonState && lastButtonState) {
            // Debounce delay
            vTaskDelay(pdMS_TO_TICKS(50));

            // Verify button is still pressed
            port_val = gpio_port_read(BUTTON_PORT);
            if (!(port_val & BUTTON_PIN)) {
                // Toggle LED state
                redLedState = !redLedState;

                uart_puts("[Button Task] Button pressed! Red LED is now: ");
                uart_puts(redLedState ? "ON\r\n" : "OFF\r\n");

                // Wait for button release
                port_val = gpio_port_read(BUTTON_PORT);
                while (!(port_val & BUTTON_PIN)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    port_val = gpio_port_read(BUTTON_PORT);
                }
            }
        }

        lastButtonState = currentButtonState;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task 3: LED control - update red LED based on state
void TaskLedControl(void *pvParameters) {
    (void) pvParameters;

    bool lastState = false;

    for (;;) {
        // Check if state changed
        if (redLedState != lastState) {
            // Take mutex before accessing LED
            if (xSemaphoreTake(xLedMutex, portMAX_DELAY) == pdTRUE) {
                if (redLedState) {
                    gpio_set(LED_RED_PORT, LED_RED_PIN);
                } else {
                    gpio_clear(LED_RED_PORT, LED_RED_PIN);
                }
                lastState = redLedState;
                xSemaphoreGive(xLedMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void) {
    // Initialize hardware
    clock_setup();
    gpio_setup();
    uart_setup();

    // Small delay for hardware to settle
    for (volatile int i = 0; i < 1000000; i++);

    uart_puts("\r\n========================================\r\n");
    uart_puts("   FreeRTOS Demo - TM4C1294XL\r\n");
    uart_puts("   libopencm3 + FreeRTOS Kernel\r\n");
    uart_puts("========================================\r\n\r\n");

    // Test LEDs on startup
    uart_puts("[Setup] Testing LEDs...\r\n");

    gpio_set(LED_RED_PORT, LED_RED_PIN);
    for (volatile int i = 0; i < 3000000; i++);
    gpio_clear(LED_RED_PORT, LED_RED_PIN);

    gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
    for (volatile int i = 0; i < 3000000; i++);
    gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);

    gpio_set(LED_BLUE_PORT, LED_BLUE_PIN);
    for (volatile int i = 0; i < 3000000; i++);
    gpio_clear(LED_BLUE_PORT, LED_BLUE_PIN);

    uart_puts("\r\n[Setup] Complete!\r\n");
    uart_puts("Creating FreeRTOS tasks...\r\n\r\n");

    // Create mutex for LED access
    xLedMutex = xSemaphoreCreateMutex();

    if (xLedMutex != NULL) {
        // Create tasks
        xTaskCreate(
            TaskHeartbeat,
            "Heartbeat",
            configMINIMAL_STACK_SIZE,
            NULL,
            2,  // Priority
            NULL
        );

        xTaskCreate(
            TaskButtonMonitor,
            "Button",
            configMINIMAL_STACK_SIZE,
            NULL,
            3,  // Higher priority for button responsiveness
            NULL
        );

        xTaskCreate(
            TaskLedControl,
            "LedCtrl",
            configMINIMAL_STACK_SIZE,
            NULL,
            2,
            NULL
        );

        uart_puts("FreeRTOS tasks created successfully!\r\n");
        uart_puts("Blue LED (D3/PF1) = Heartbeat (blinks every second)\r\n");
        uart_puts("SW1 Button (PJ0) = Toggle Red LED (D1/PF0)\r\n\r\n");
        uart_puts("Starting FreeRTOS scheduler...\r\n\r\n");

        // Start the scheduler
        vTaskStartScheduler();

        // Should never reach here
        uart_puts("ERROR: Scheduler failed to start!\r\n");
    } else {
        uart_puts("ERROR: Failed to create mutex!\r\n");
    }

    // If we get here, something went wrong
    while (1) {
        gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        for (volatile int i = 0; i < 1000000; i++);
    }
}

// FreeRTOS hooks
extern "C" {
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
        (void) xTask;
        (void) pcTaskName;
        // Stack overflow - blink all LEDs rapidly
        while (1) {
            gpio_toggle(LED_RED_PORT, LED_RED_PIN);
            gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
            gpio_toggle(LED_BLUE_PORT, LED_BLUE_PIN);
            for (volatile int i = 0; i < 500000; i++);
        }
    }

    void vApplicationMallocFailedHook(void) {
        // Malloc failed - blink red LED rapidly
        while (1) {
            gpio_toggle(LED_RED_PORT, LED_RED_PIN);
            for (volatile int i = 0; i < 500000; i++);
        }
    }
}
