#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <pthread.h>
#include <pigpio.h>

// ADXL345 registers
#define POWER_CTL   0x2D    // power mode register address
#define DATA_FORMAT 0x31    // data format register address
#define BW_RATE     0x2C    // baud rate register address
#define INT_ENABLE  0x2E    // enable interrupt register address
#define INT_MAP     0x2F    // map interrupt pin register address
#define INT_SOURCE  0x30    // interrupt state register address
#define FIFO_CTL    0x38    // FIFO mode register address
#define DATAX0      0x32    // Data X0 register address
#define FIFO_STATUS 0x39    // FIFO status register address
#define WATERMARK_SIZE 16   // Watermark sample size

#define WATERMARK_PIN 27    // Watermark interrupt pin
// #define WATERMARK_PIN 7    // Watermark interrupt pin

const double conversion_const = (2.0*16.0)/8192;    // +-16g for 13 bits, pg 27
const int rw_bit = 7;                               // Read/Write bit
const int multi_byte_bit = 6;                       // Mutiple byte bit
const int fifo_size = 50;                           // Max number of saved values, FIFO(32) + extra space

// SPI comm configuration
const int spi_speed = 5000000;                      // SPI communication speed, bps
const int spi_channel = 0;                          // SPI communication channel
const int spi_buffer_size = 7;                      // Communication buffer size
const double delay_read = 0.006;                    // Delay for reading FIFO update

// Buffer handling
pthread_mutex_t bufferAMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t bufferBMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t bufferCMutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct data_buffer {
    int samples_counter;
    unsigned long int * number_data;
    int * overrun_data;
    double * x_data;
    double * y_data;
    double * z_data;
    double * block_time_data;
    double * sample_time_data;
} data_buffer;

typedef struct buffer_handler{
    struct timespec start_time;
    data_buffer bufferA;
    data_buffer bufferB;
    data_buffer bufferC;
    unsigned long int sample_number;    // Samples total number to sort order
    int updated_A;        // Indicates a new reading have been done in buffer A
    int updated_B;        // Indicates a new reading have been done in buffer B
    int updated_C;        // Indicates a new reading have been done in buffer C
    int spi_handle;
} buffer_handler;

enum { NS_PER_SECOND = 1000000000,
       MS_PER_NS = 1000000,
       MS_PER_SECOND = 1000
};

double time_delta_now(struct timespec t1){
    struct timespec t2;
    struct timespec tdelta;

    clock_gettime(CLOCK_REALTIME, &t2);

    tdelta.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    tdelta.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (tdelta.tv_sec > 0 && tdelta.tv_nsec < 0)
    {
        tdelta.tv_nsec += NS_PER_SECOND;
        tdelta.tv_sec--;
    }
    else if (tdelta.tv_sec < 0 && tdelta.tv_nsec > 0)
    {
        tdelta.tv_nsec -= NS_PER_SECOND;
        tdelta.tv_sec++;
    }

    return (((double)tdelta.tv_sec*NS_PER_SECOND) + (double)tdelta.tv_nsec)/NS_PER_SECOND;
}

void delay_ms(double ms){
    struct timespec delay;
    delay.tv_sec = (int)(ms/MS_PER_SECOND);
    delay.tv_nsec = (long int)((ms - (int)(ms/MS_PER_SECOND)*MS_PER_SECOND)*MS_PER_NS);
    nanosleep(&delay,NULL);
}

int spi_read(int spi_handle, char *command, char *buffer, int count){
    command[0] |= (1 << rw_bit);
    if(count > 1){
        command[0] |= (1 << multi_byte_bit);
    }
    return spiXfer(spi_handle, command, buffer, count);
}

int spi_write(int spi_handle, char *buffer, int count){
    if(count > 1){
        buffer[0] |= (1 << multi_byte_bit);
    }
    return spiWrite(spi_handle, buffer, count);
}

void clear_fifo(int spi_handle){
    char command[2];
    char config = 0;

    command[0] = FIFO_CTL;

    // Saving old configuration
    spi_read(spi_handle, command, command, 2);
    config = command[1];

    // Clearing FIFO
    command[1] = 0;
    spi_write(spi_handle, command, 2);
    delay_ms(10);

    // Getting old configuration back
    command[1] = config;
    spi_write(spi_handle, command, 2);
}

int fifo_status(int spi_handle){
    char buffer[spi_buffer_size];

    buffer[0] = FIFO_STATUS;
    spi_read(spi_handle, buffer, buffer, 2);

    return (int)(buffer[1] & 0x3F);
}

void clear_fifo_forced(int spi_handle){
    char buffer[spi_buffer_size];

    while(fifo_status(spi_handle)){
        buffer[0] = DATAX0;
        spi_read(spi_handle, buffer, buffer, spi_buffer_size);
        delay_ms(0.006);
    }
}

void configure_adxl(int spi_handle){
    char command[2];

    // Set Data Format
    command[0] = DATA_FORMAT;
    command[1] = 0x0B;
    // SELF_TEST | SPI | INT_INVERT | 0 | FULL_RES | Justify | Range |
    //      0    |  0  |      0     | 0 |     1    |    0    |  1 1  |
    //     OFF   |4-wir|active high | 0 |  13-bit  | right-j | +-16g |
    // (datasheet ADXL345, pg 27)
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // Set Sample Rate
    command[0] = BW_RATE;
    command[1] = 0x0F;
    // 0 | 0 | 0 | LOW_POWER | Rate
    // 0 | 0 | 0 |     0     | 1111
    // 0 | 0 | 0 |    OFF    | 3200Hz
    // (datasheet ADXL345, pg 25)
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // Set interrupt pin map
    command[0] = INT_MAP;
    command[1] = 0x01;
    // DATA_READY | SINGLE_TAP | DOUBLE_TAP | Activity | Inactivity | FREE_FALL | Watermark | Overrun
    //      0     |      0     |      0     |     0    |      0     |     0     |     0     |    1
    //    INT1    |    INT1    |    INT1    |   INT1   |    INT1    |   INT1    |    INT1   |   INT2
    // (datasheet ADXL345, pg 26)
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // FIFO Mode and Watermark sample size
    command[0] = FIFO_CTL;
    command[1] = 0xA0 | WATERMARK_SIZE;
    // FIFO_MODE | Trigger | Samples
    //    1 0    |    1    | x x x x x
    //  stream   |   INT2  | x samples
    // (datasheet ADXL345, pg 28)
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // Enable interrupt functions
    command[0] = INT_ENABLE;
    command[1] = 0x03;
    // DATA_READY | SINGLE_TAP | DOUBLE_TAP | Activity | Inactivity | FREE_FALL | Watermark | Overrun
    //      0     |      0     |      0     |     0    |      0     |     0     |     1     |    1
    //     off    |     off    |     off    |    off   |     off    |    off    | activated | activated
    // (datasheet ADXL345, pg 26);
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // Set Power Mode
    command[0] = POWER_CTL;
    command[1] = 0x08;
    // 0 | 0 | Link | AUTO_SLEEP | measure | Sleep | Wakeup |
    // 0 | 0 |  0   |      0     |    1    |   0   |   0 0  |
    // 0 | 0 | conc | deactivated| active  | normal|   off  |
    // (datasheet ADXL345, pg 26)
    spi_write(spi_handle, command, 2);
    delay_ms(1);
}

int verify_spi(int spi_handle){
    if(spi_handle < 0){
        switch(spi_handle){
            case(PI_BAD_SPI_CHANNEL):
                printf("Wrong SPI Channel.");
                break;
            case(PI_BAD_SPI_SPEED):
                printf("Wrong SPI Baud Rate.");
                break;
            case(PI_BAD_FLAGS):
                printf("Bad Flags.");
                break;
            case(PI_NO_AUX_SPI):
                printf("No Auxiliary SPI.");
                break;
            case(PI_SPI_OPEN_FAILED):
                printf("Failed to open SPI.");
                break;
        }
    }
    return spi_handle;
}

void trigger_status(int spi_handle, int *watermark_trigger,int *overrun_trigger){
    char buffer[spi_buffer_size];

    buffer[0] = INT_SOURCE;

    spi_read(spi_handle, buffer, buffer, 2);
    *watermark_trigger = (buffer[1] & 0x02)>>1;
    *overrun_trigger = (buffer[1] & 0x01);
}

void save_data(char *output_name, data_buffer buffer){
    // Save data
    FILE * pFile;
    pFile = fopen(output_name, "a");
    for(int i=0; i < buffer.samples_counter; i++){
        fprintf(pFile, "%ld, %.9f, %.9f, %d, %.9f, %.9f, %.9f\n",
                buffer.number_data[i],
                buffer.block_time_data[i], buffer.sample_time_data[i],
                buffer.overrun_data[i],
                buffer.x_data[i], buffer.y_data[i], buffer.z_data[i]);
    }
    fclose(pFile);
}

void save_FIFO_values(buffer_handler *handler, data_buffer *buffer, int sampled_values, double t_block){
    char spi_buffer[spi_buffer_size];
    char command[spi_buffer_size];
    double t_sample;
    int16_t x, y, z;
    int result;

    for(int i=0; i<sampled_values; i++){
        command[0] = DATAX0;
        result = spi_read(handler->spi_handle, command, spi_buffer, spi_buffer_size);
        if(result < spi_buffer_size){
            printf("Buffer read size {%d} is smaller than expected {%d}.\n", result, spi_buffer_size);
        }
        else{
            t_sample = time_delta_now(handler->start_time);
            x = (spi_buffer[2]<<8) | spi_buffer[1];
            y = (spi_buffer[4]<<8) | spi_buffer[3];
            z = (spi_buffer[6]<<8) | spi_buffer[5];

            buffer->number_data[buffer->samples_counter] = handler->sample_number;
            buffer->block_time_data[buffer->samples_counter] = t_block;
            buffer->sample_time_data[buffer->samples_counter] = t_sample;
            buffer->overrun_data[buffer->samples_counter] = 0;
            buffer->x_data[buffer->samples_counter] = (double)x * conversion_const;
            buffer->y_data[buffer->samples_counter] = (double)y * conversion_const;
            buffer->z_data[buffer->samples_counter] = (double)z * conversion_const;

            buffer->samples_counter = buffer->samples_counter + 1;
            handler->sample_number = handler->sample_number + 1;
        }
        delay_ms(0.005);
    }
}

void watermarkInterruptHandler(int gpio, int level, uint32_t tick, void * exData) {
    char spi_buffer[spi_buffer_size];
    char command[spi_buffer_size];
    int16_t x, y, z;
    int result;
    double t_sample,t_block;
    int selected;                   // Selected buffer
    int sampled_values;

    buffer_handler *handler = (buffer_handler *)exData;
    t_block = time_delta_now(handler->start_time);

    if(handler->updated_A == 0){
        if(pthread_mutex_trylock(&bufferAMutex) == 0)   selected = 1;
        else printf("\nFailed to open buffer A\n");
    }
    else if(handler->updated_B == 0){
        if(pthread_mutex_trylock(&bufferBMutex) == 0)   selected = 2;
        else printf("\nFailed to open buffer B\n");
    }
    else if(handler->updated_C == 0){
        if(pthread_mutex_trylock(&bufferCMutex) == 0)   selected = 3;
        else printf("\nFailed to open buffer C\n");
    }
    else return;

    if (selected == 1){
        save_FIFO_values(handler, &handler->bufferA, WATERMARK_SIZE, t_block);
        delay_ms(0.006);
        sampled_values = fifo_status(handler->spi_handle);
        save_FIFO_values(handler, &handler->bufferA, sampled_values, t_block);
        pthread_mutex_unlock(&bufferAMutex);
        handler->updated_A = 1;
    }
    if (selected == 2){
        save_FIFO_values(handler, &handler->bufferB, WATERMARK_SIZE, t_block);
        delay_ms(0.006);
        sampled_values = fifo_status(handler->spi_handle);
        save_FIFO_values(handler, &handler->bufferB, sampled_values, t_block);
        pthread_mutex_unlock(&bufferBMutex);
        handler->updated_B = 1;
    }
    if (selected == 3){
        save_FIFO_values(handler, &handler->bufferC, WATERMARK_SIZE, t_block);
        delay_ms(0.006);
        sampled_values = fifo_status(handler->spi_handle);
        save_FIFO_values(handler, &handler->bufferC, sampled_values, t_block);
        pthread_mutex_unlock(&bufferCMutex);
        handler->updated_C = 1;
    }
}

int main(int argc, char *argv[]) {
    buffer_handler handler;
    double sample_time = 5;              // sample time in seconds
    int sample_rate = 3200;                 // sample rate in Hz
    char output_name[256] = "data.csv";

    // Allocate space for data
    data_buffer bufferA = {
        0, // samples_counter
        malloc(fifo_size * sizeof(unsigned long int)),  // number_data
        malloc(fifo_size * sizeof(int)),                // overrun_data
        malloc(fifo_size * sizeof(double)),             // x_data
        malloc(fifo_size * sizeof(double)),             // y_data
        malloc(fifo_size * sizeof(double)),             // z_data
        malloc(fifo_size * sizeof(double)),             // block_time_data
        malloc(fifo_size * sizeof(double)),             // sample_time_data
    };
    data_buffer bufferB = {
        0, // samples_counter
        malloc(fifo_size * sizeof(unsigned long int)),  // number_data
        malloc(fifo_size * sizeof(int)),                // overrun_data
        malloc(fifo_size * sizeof(double)),             // x_data
        malloc(fifo_size * sizeof(double)),             // y_data
        malloc(fifo_size * sizeof(double)),             // z_data
        malloc(fifo_size * sizeof(double)),             // block_time_data
        malloc(fifo_size * sizeof(double)),             // sample_time_data
    };
    data_buffer bufferC = {
        0, // samples_counter
        malloc(fifo_size * sizeof(unsigned long int)),  // number_data
        malloc(fifo_size * sizeof(int)),                // overrun_data
        malloc(fifo_size * sizeof(double)),             // x_data
        malloc(fifo_size * sizeof(double)),             // y_data
        malloc(fifo_size * sizeof(double)),             // z_data
        malloc(fifo_size * sizeof(double)),             // block_time_data
        malloc(fifo_size * sizeof(double)),             // sample_time_data
    };
    handler.bufferA = bufferA;
    handler.bufferB = bufferB;
    handler.bufferC = bufferC;

    // Create Output file, and put header
    FILE * pFile;
    pFile = fopen(output_name, "w");
    fprintf(pFile, "sample_number,block_time,sample_time,overrun,x,y,z\n");
    fclose(pFile);

    // Starting GPIO
    if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
        printf("GPIO initialization failed.");
        return 1;
    }
    // Starting mutex
    pthread_mutex_init(&bufferAMutex, NULL);
    pthread_mutex_init(&bufferBMutex, NULL);
    pthread_mutex_init(&bufferCMutex, NULL);

    // Starting SPI
    handler.spi_handle = spiOpen(spi_channel, spi_speed, 3);
    if(verify_spi(handler.spi_handle) < 0){ return 1; }

    configure_adxl(handler.spi_handle);

    // Starting acquisition
    printf("Sample Time: %.6f seconds\n",sample_time);

    handler.updated_A = 0;
    handler.updated_B = 0;
    handler.updated_C = 0;
    handler.sample_number = 0;
    clear_fifo_forced(handler.spi_handle);

    // Set up the interrupt handler
    if (gpioSetAlertFuncEx(WATERMARK_PIN, &watermarkInterruptHandler, &handler) < 0) {
        fprintf(stderr, "Error setting up interrupt\n");
        gpioTerminate();
        return 1;
    }
    gpioSetPullUpDown(WATERMARK_PIN, PI_PUD_UP);

    delay_ms(1/((double)sample_rate*2));

    clock_gettime(CLOCK_REALTIME, &(handler.start_time));

    while(time_delta_now(handler.start_time) < sample_time){
        if(handler.updated_A){
            if(pthread_mutex_trylock(&bufferAMutex) == 0){
                save_data(output_name,handler.bufferA);

                handler.updated_A = 0;
                handler.bufferA.samples_counter = 0;

                pthread_mutex_unlock(&bufferAMutex);
            }
        }

        if(handler.updated_B){
            if(pthread_mutex_trylock(&bufferBMutex) == 0){
            save_data(output_name,handler.bufferB);

            handler.updated_B = 0;
            handler.bufferB.samples_counter = 0;

            pthread_mutex_unlock(&bufferBMutex);
            }
        }

        if(handler.updated_C){
            if(pthread_mutex_trylock(&bufferCMutex) == 0){
                save_data(output_name,handler.bufferC);

                handler.updated_C = 0;
                handler.bufferC.samples_counter = 0;

                pthread_mutex_unlock(&bufferCMutex);
            }
        }

        delay_ms(MS_PER_SECOND/(double)sample_rate);
    }

    printf("\nElapsed Time: %.6f seconds\n", time_delta_now(handler.start_time));
    printf("Samples Quantity: %ld\n", handler.sample_number);

    // Finishing SPI and GPIO
    spiClose(handler.spi_handle);
    gpioTerminate();

    free(bufferA.x_data);
    free(bufferA.y_data);
    free(bufferA.z_data);
    free(bufferA.number_data);
    free(bufferA.block_time_data);
    free(bufferA.sample_time_data);
    free(bufferA.overrun_data);

    free(bufferB.x_data);
    free(bufferB.y_data);
    free(bufferB.z_data);
    free(bufferB.number_data);
    free(bufferB.block_time_data);
    free(bufferB.sample_time_data);
    free(bufferB.overrun_data);

    free(bufferC.x_data);
    free(bufferC.y_data);
    free(bufferC.z_data);
    free(bufferC.number_data);
    free(bufferC.block_time_data);
    free(bufferC.sample_time_data);
    free(bufferC.overrun_data);

    pthread_mutex_destroy(&bufferAMutex);
    pthread_mutex_destroy(&bufferBMutex);
    pthread_mutex_destroy(&bufferCMutex);

    printf("Done\n");
    return 0;
}
