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

const double conversion_const = (2.0*16.0)/8192;       // +-16g for 13 bits, pg 27
const int rw_bit = 7;                               // Read/Write bit
const int multi_byte_bit = 6;                       // Mutiple byte bit
const int fifo_size = 33;                           // Max number of saved values, FIFO + last reading

// SPI comm configuration
const int spi_speed = 5000000;                      // SPI communication speed, bps
const int spi_channel = 0;                          // SPI communication channel
const int spi_buffer_size = 7;                      // Communication buffer size
const double delay_read = 0.006;                    // Delay for reading FIFO update

// Buffer handling
pthread_mutex_t bufferAMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t bufferBMutex = PTHREAD_MUTEX_INITIALIZER;

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
    int overrun_trigger;
    data_buffer bufferA;
    // data_buffer bufferB;
    unsigned long int sample_number_A;    // Samples total number to sort order
    // unsigned long int sample_number_B;    // Samples total number to sort order
    // int racing;         // Indicates the racing condition happened
    int updated;        // Indicates a new reading have been done
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

    return buffer[1] & 0x3F;
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
    command[1] = 0x02;
    // DATA_READY | SINGLE_TAP | DOUBLE_TAP | Activity | Inactivity | FREE_FALL | Watermark | Overrun
    //      0     |      0     |      0     |     0    |      0     |     0     |     1     |    0
    //    INT1    |    INT1    |    INT1    |   INT1   |    INT1    |   INT1    |    INT2   |   INT1
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
    buffer.samples_counter = 0;
}

// void watermarkInterruptHandler(int gpio, int level, uint32_t tick, void * exData) {
void watermarkInterruptHandler(void * exData) {
    char spi_buffer[spi_buffer_size];
    char command[spi_buffer_size];
    int16_t x, y, z;
    int result;
    double t_sample;
    double t_block;

    buffer_handler *handler = (buffer_handler *)exData;

    t_block = time_delta_now(handler->start_time);
    pthread_mutex_lock(&bufferAMutex);

    for(int i=0; i<WATERMARK_SIZE; i++){
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

            handler->bufferA.number_data[handler->bufferA.samples_counter] = handler->sample_number_A;
            handler->bufferA.block_time_data[handler->bufferA.samples_counter] = t_block;
            handler->bufferA.sample_time_data[handler->bufferA.samples_counter] = t_sample;
            handler->bufferA.overrun_data[handler->bufferA.samples_counter] = handler->overrun_trigger;
            handler->bufferA.x_data[handler->bufferA.samples_counter] = (double)x * conversion_const;
            handler->bufferA.y_data[handler->bufferA.samples_counter] = (double)y * conversion_const;
            handler->bufferA.z_data[handler->bufferA.samples_counter] = (double)z * conversion_const;

            handler->bufferA.samples_counter++;
            handler->sample_number_A++;
        }
        delay_ms(0.005);
    }
    handler->updated = 1;
    pthread_mutex_unlock(&bufferAMutex);
}

int main(int argc, char *argv[]) {
    int watermark_trigger, overrun_trigger;
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
    handler.bufferA = bufferA;

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

    // Starting SPI
    handler.spi_handle = spiOpen(spi_channel, spi_speed, 3);
    if(verify_spi(handler.spi_handle) < 0){ return 1; }

    configure_adxl(handler.spi_handle);

    // Starting acquisition
    printf("Sample Time: %.6f seconds\n",sample_time);

    handler.updated = 0;
    handler.sample_number_A = 0;
    clear_fifo_forced(handler.spi_handle);
    delay_ms(1/(double)sample_rate);

    clock_gettime(CLOCK_REALTIME, &(handler.start_time));

    while(time_delta_now(handler.start_time) < sample_time){
        fifo_status(handler.spi_handle);
        trigger_status(handler.spi_handle, &watermark_trigger, &overrun_trigger);
        handler.overrun_trigger = overrun_trigger;

        if(overrun_trigger){
            printf("Overrun!");
            printf("Delta: %f\n", time_delta_now(handler.start_time));
        }

        if(watermark_trigger>0){
            watermarkInterruptHandler(&handler);
        }
        if(handler.updated == 1){
            save_data(output_name,
                      handler.bufferA);
            handler.bufferA.samples_counter = 0;
            handler.updated = 0;
        }
        delay_ms(MS_PER_SECOND/((double)sample_rate*2));
    }

    printf("\nElapsed Time: %.6f seconds\n", time_delta_now(handler.start_time));
    printf("Samples Quantity: %ld\n", handler.sample_number_A);

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

    printf("Done\n");
    return 0;
}