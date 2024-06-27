#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include <pigpio.h>

#define GPIO_PIN 25 // Replace with the GPIO pin you are monitoring
#define MAX_TIMESTAMPS 100

time_t timestampVector[MAX_TIMESTAMPS];
int timestampIndex = 0;
pthread_mutex_t vectorMutex = PTHREAD_MUTEX_INITIALIZER;

void interruptHandler(int gpio, int level, uint32_t tick) {
    time_t currentTime;
    time(&currentTime);

    // Lock the mutex before accessing the shared array
    pthread_mutex_lock(&vectorMutex);

    // Update the shared array
    if (timestampIndex < MAX_TIMESTAMPS) {
        timestampVector[timestampIndex++] = currentTime;
    }

    // Unlock the mutex after updating the shared array
    pthread_mutex_unlock(&vectorMutex);

    // ... (rest of the interrupt handling code)
}

void processTimestamps() {
    // Lock the mutex before accessing the shared array
    pthread_mutex_lock(&vectorMutex);

    // Process timestampVector as needed
    for (int i = 0; i < timestampIndex; ++i) {
        printf("Timestamp: %s", ctime(&timestampVector[i]));
    }
    timestampIndex = 0; // Reset the index after processing

    // Unlock the mutex after processing the shared array
    pthread_mutex_unlock(&vectorMutex);
}

int main() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Error initializing pigpio\n");
        return 1;
    }

    // Set up the interrupt handler
    if (gpioSetAlertFunc(GPIO_PIN, &interruptHandler) < 0) {
        fprintf(stderr, "Error setting up interrupt\n");
        gpioTerminate();
        return 1;
    }

    printf("Monitoring GPIO pin %d for interruptions...\n", GPIO_PIN);
    gpioSetPullUpDown(GPIO_PIN, PI_PUD_UP);
    // Keep the program running
    while (1) {
        // Access the shared array in the main function
        processTimestamps();

        gpioDelay(1000000); // Delay to keep the program running (1 second)
    }

    // Clean up
    gpioTerminate();

    return 0;
}
