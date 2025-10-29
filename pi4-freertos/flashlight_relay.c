#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/select.h>

/* FreeRTOS-like task management */
#define TASK_STACK_SIZE 2048
#define PRIORITY_HIGH 2
#define PRIORITY_NORMAL 1
#define PRIORITY_LOW 0

/* Command definitions */
#define CMD_BUTTON_PRESSED "BTN_PRESSED"
#define CMD_LED_TOGGLE "LED_TOGGLE"
#define CMD_LED_ACK "LED_OK"

/* Global state */
typedef struct {
    int serial_fd;
    char buffer[256];
    int buffer_pos;
    pthread_mutex_t mutex;
    sem_t button_event;
    int button_pressed;
} SharedState;

SharedState shared_state = {
    .serial_fd = -1,
    .buffer_pos = 0,
    .mutex = PTHREAD_MUTEX_INITIALIZER,
    .button_event = {{0}},
    .button_pressed = 0
};

/* Initialize serial port */
int init_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    /* Set baudrate */
    speed_t speed;
    switch (baudrate) {
        case 115200: speed = B115200; break;
        default: speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* 8N1 */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

/* Flush serial input buffer (discard any startup messages) */
void flush_serial_buffer(int fd) {
    char throwaway[256];
    ssize_t bytes;
    int count = 0;
    
    printf("[Setup] Clearing serial buffer...\n");
    fflush(stdout);
    
    while ((bytes = read(fd, throwaway, sizeof(throwaway))) > 0 && count < 100) {
        count++;
        usleep(10000);  /* 10ms - let more data arrive */
    }
    
    if (count > 0) {
        printf("[Setup] Discarded %d bytes of initial data\n", count);
        fflush(stdout);
    }
    
    usleep(500000);  /* 500ms - wait for Tiva to finish startup */
}

/* Send command over serial */
void send_command(const char *cmd) {
    pthread_mutex_lock(&shared_state.mutex);
    if (shared_state.serial_fd >= 0) {
        char buffer[512];
        snprintf(buffer, sizeof(buffer), "%s\n", cmd);
        ssize_t written = write(shared_state.serial_fd, buffer, strlen(buffer));
        if (written < 0) {
            perror("write");
        } else {
            printf("[Pi4] Sent: %s\n", cmd);
            fflush(stdout);
        }
    }
    pthread_mutex_unlock(&shared_state.mutex);
}

/* Serial receive task - reads from Tiva */
void* task_serial_receiver(void *arg) {
    printf("[Task] Serial Receiver started\n");
    fflush(stdout);
    
    while (1) {
        pthread_mutex_lock(&shared_state.mutex);
        if (shared_state.serial_fd >= 0) {
            char byte;
            ssize_t n = read(shared_state.serial_fd, &byte, 1);
            
            if (n > 0) {
                if (byte == '\n') {
                    shared_state.buffer[shared_state.buffer_pos] = '\0';
                    
                    /* Only process if we have actual command data (simple validation) */
                    if (strlen(shared_state.buffer) > 3) {
                        printf("[Serial] Received: '%s'\n", shared_state.buffer);
                        fflush(stdout);
                        
                        if (strcmp(shared_state.buffer, CMD_BUTTON_PRESSED) == 0) {
                            shared_state.button_pressed = 1;
                            sem_post(&shared_state.button_event);
                        } else if (strcmp(shared_state.buffer, CMD_LED_ACK) == 0) {
                            printf("[Pi4] LED toggle acknowledged by Tiva\n");
                            fflush(stdout);
                        }
                    }
                    
                    shared_state.buffer_pos = 0;
                } else if (byte != '\r' && shared_state.buffer_pos < (int)sizeof(shared_state.buffer) - 1) {
                    shared_state.buffer[shared_state.buffer_pos++] = byte;
                }
            }
        }
        pthread_mutex_unlock(&shared_state.mutex);
        
        usleep(10000);  /* 10ms sleep */
    }
    
    return NULL;
}

/* LED control task - waits for button events and sends LED_TOGGLE */
void* task_led_controller(void *arg) {
    printf("[Task] LED Controller started\n");
    fflush(stdout);
    
    sem_init(&shared_state.button_event, 0, 0);
    
    while (1) {
        printf("[LED Controller] Waiting for button press...\n");
        fflush(stdout);
        if (sem_wait(&shared_state.button_event) == 0) {
            printf("[LED Controller] Button event received!\n");
            fflush(stdout);
            
            /* Decision logic - send LED_TOGGLE command */
            sleep(1);  /* Small delay to simulate processing */
            send_command(CMD_LED_TOGGLE);
        }
    }
    
    return NULL;
}

/* Heartbeat task - keeps system alive */
void* task_heartbeat(void *arg) {
    printf("[Task] Heartbeat started\n");
    fflush(stdout);
    int count = 0;
    
    while (1) {
        printf("[Pi4] System alive (tick %d)\n", ++count);
        fflush(stdout);
        sleep(2);
    }
    
    return NULL;
}

int main(int argc, char *argv[]) {
    printf("\n========================================\n");
    printf("  Flashlight UI - FreeRTOS Relay\n");
    printf("  Raspberry Pi 4 (Task Scheduler)\n");
    printf("========================================\n\n");
    fflush(stdout);

    /* Find serial device - ICDI appears as ttyACM0 on Linux */
    const char *serial_device = "/dev/ttyACM1";
    
    printf("[Main] Connecting to Tiva on %s...\n", serial_device);
    fflush(stdout);
    shared_state.serial_fd = init_serial(serial_device, 115200);
    
    if (shared_state.serial_fd < 0) {
        fprintf(stderr, "Error: Could not open serial port %s\n", serial_device);
        fprintf(stderr, "Make sure the ICDI USB cable is connected to the Pi4\n");
        fprintf(stderr, "\nTroubleshooting:\n");
        fprintf(stderr, "  1. Check: ls /dev/ttyACM*\n");
        fprintf(stderr, "  2. Check permissions: ls -l /dev/ttyACM0\n");
        fprintf(stderr, "  3. Try: sudo chmod 666 /dev/ttyACM0\n");
        fprintf(stderr, "  4. Check kernel: dmesg | tail\n");
        return 1;
    }
    
    printf("[Main] Serial connection established!\n");
    fflush(stdout);

    /* Flush serial buffer to clear startup messages from Tiva */
    flush_serial_buffer(shared_state.serial_fd);
    
    printf("[Main] Serial buffer cleared, ready to receive commands\n\n");
    fflush(stdout);

    /* Create FreeRTOS-style tasks */
    pthread_t task_serial, task_led, task_hb;
    
    printf("[Main] Creating tasks...\n");
    fflush(stdout);
    pthread_create(&task_serial, NULL, task_serial_receiver, NULL);
    pthread_create(&task_led, NULL, task_led_controller, NULL);
    pthread_create(&task_hb, NULL, task_heartbeat, NULL);
    
    printf("[Main] All tasks created. System running.\n");
    printf("[Main] Press button on Tiva to trigger LED toggle\n\n");
    fflush(stdout);

    /* Keep main thread alive */
    pthread_join(task_serial, NULL);
    
    close(shared_state.serial_fd);
    return 0;
}
