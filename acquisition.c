#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

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

const double conversion_const = (2 * 2.0)/1024.0;   // Conversion
const int rw_bit = 7;                               // Read/Write bit
const int multi_byte_bit = 6;                       // Mutiple byte bit

// SPI comm configuration
const int spi_speed = 2000000;                      // SPI communication speed, bps
const int spi_channel = 0;                          // SPI communication channel
const int spi_buffer_size = 7;                      // Communication buffer size
const double delay_read = 0.006;                    // Delay for reading FIFO update

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

void clear_fifo(int spi_handle){
    char command[2];
    char config = 0;

    command[0] = FIFO_CTL;

    // Saving old configuration
    spi_read(spi_handle, command, command, 2);
    config = command[1];

    // Clearing FIFO
    command[1] = 0;
    spiWrite(spi_handle, command, 2);
    delay_ms(10);

    // Getting old configuration back
    command[1] = config;
    spiWrite(spi_handle, command, 2);
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
    command[1] = 0x00;
    // SELF_TEST | SPI | INT_INVERT | 0 | FULL_RES | Justify | Range |
    //      0    |  0  |      0     | 0 |     0    |    0    |  0 0  |
    //     OFF   |4-wir|active high | 0 |  10-bit  | right-j | +- 2g |
    // (datasheet ADXL345, pg 27)
    spiWrite(spi_handle, command, 2);
    delay_ms(1);

    // Set Sample Rate
    command[0] = BW_RATE;
    command[1] = 0x0F;
    // 0 | 0 | 0 | LOW_POWER | Rate
    // 0 | 0 | 0 |     0     | 1111
    // 0 | 0 | 0 |    OFF    | 3200Hz
    // (datasheet ADXL345, pg 25)
    spiWrite(spi_handle, command, 2);
    delay_ms(1);

    // Set interrupt pin map
    command[0] = INT_MAP;
    command[1] = 0x02;
    // DATA_READY | SINGLE_TAP | DOUBLE_TAP | Activity | Inactivity | FREE_FALL | Watermark | Overrun
    //      0     |      0     |      0     |     0    |      0     |     0     |     1     |    0
    //    INT1    |    INT1    |    INT1    |   INT1   |    INT1    |   INT1    |    INT2   |   INT1
    // (datasheet ADXL345, pg 26)
    spiWrite(spi_handle, command, 2);
    delay_ms(1);

    // FIFO Mode and Watermark sample size
    command[0] = FIFO_CTL;
    command[1] = 0xB0;
    spiWrite(spi_handle, command, 2);
    // FIFO_MODE | Trigger | Samples
    //    1 0    |    1    | 1 0 0 0 0
    //  stream   |   INT2  | 16 samples
    // (datasheet ADXL345, pg 28)
    // spiWrite(spi_handle, command, 2);
    delay_ms(1);

    // Enable interrupt functions
    command[0] = INT_ENABLE;
    command[1] = 0x03;
    // DATA_READY | SINGLE_TAP | DOUBLE_TAP | Activity | Inactivity | FREE_FALL | Watermark | Overrun
    //      0     |      0     |      0     |     0    |      0     |     0     |     1     |    1
    //     off    |     off    |     off    |    off   |     off    |    off    | activated | activated
    // (datasheet ADXL345, pg 26);
    spiWrite(spi_handle, command, 2);
    delay_ms(1);

    // Set Power Mode
    command[0] = POWER_CTL;
    command[1] = 0x08;
    // 0 | 0 | Link | AUTO_SLEEP | measure | Sleep | Wakeup |
    // 0 | 0 |  0   |      0     |    1    |   0   |   0 0  |
    // 0 | 0 | conc | deactivated| active  | normal|   off  |
    // (datasheet ADXL345, pg 26)
    spiWrite(spi_handle, command, 2);
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

int main(int argc, char *argv[]) {
    char buffer[spi_buffer_size];
    int spi_handle;
    int result, sampled_values, watermark_trigger, overrun_trigger;
    int x, y, z, t;
    double *time_data, *x_data, *y_data, *z_data;
    struct timespec start_time;

    int samples_counter = 0;    // counter of samples
    double sample_time = 5;       // sample time in seconds
    int sample_rate = 3200;     // sample rate in Hz
    char output_name[256] = "data.csv";

    // Allocate space for data
    x_data = malloc(sample_time * sample_rate * sizeof(double) * 20);
    y_data = malloc(sample_time * sample_rate * sizeof(double) * 20);
    z_data = malloc(sample_time * sample_rate * sizeof(double) * 20);
    time_data = malloc(sample_time * sample_rate * sizeof(double) * 20);

    // Starting GPIO
    if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
        printf("GPIO initialization failed.");
        return 1;
    }

    // Starting SPI
    spi_handle = spiOpen(spi_channel, spi_speed, 3);
    if(verify_spi(spi_handle) < 0){ return 1; }

    configure_adxl(spi_handle);

    // Starting acquisition
    printf("Sample Time: %.6f seconds\n",sample_time);
    printf("Sample Delay time: %.6f\n", 1/(double)sample_rate);

    clear_fifo_forced(spi_handle);
    delay_ms(1/(double)sample_rate);

    clock_gettime(CLOCK_REALTIME, &start_time);

    printf("Delta: %f\n", time_delta_now(start_time));
    while(time_delta_now(start_time) < sample_time){
        sampled_values = fifo_status(spi_handle);
        trigger_status(spi_handle, &watermark_trigger, &overrun_trigger);

        if(overrun_trigger){
            printf("Overrun!");
            printf("Delta: %f\n", time_delta_now(start_time));
        }

        if((result>0) && (watermark_trigger>0)){
            // printf("Delta: %f\n", time_delta_now(start_time));
            for(int i=0; i<sampled_values; i++,delay_ms(0.006)){
                buffer[0] = DATAX0;
                result = spi_read(spi_handle, buffer, buffer, spi_buffer_size);
                if(result < spi_buffer_size){
                    printf("Buffer read size {%d} is smaller than expected {%d}.\n", result, spi_buffer_size);
                }
                else{
                    x = (buffer[2]<<8)|buffer[1];
                    y = (buffer[4]<<8)|buffer[3];
                    z = (buffer[6]<<8)|buffer[5];
                    t = time_delta_now(start_time);

                    x_data[samples_counter] = x*conversion_const;
                    y_data[samples_counter] = y*conversion_const;
                    z_data[samples_counter] = z*conversion_const;
                    time_data[samples_counter] = t;
                    samples_counter+=1;
                }
            }
        }
        delay_ms(1000/(double)sample_rate);
    }

    printf("\nElapsed Time: %.6f seconds\n", time_delta_now(start_time));
    printf("Samples Quantity: %d\n", samples_counter);

    // Finishing SPI and GPIO
    spiClose(spi_handle);
    gpioTerminate();

    // Save data
    FILE * pFile;
    pFile = fopen(output_name, "w");
    fprintf(pFile, "time, x, y, z\n");
    for(int i=0; i <= samples_counter; i++){
        fprintf(pFile, "%.4f, %.4f, %.4f, %.4f\n", time_data[i], x_data[i], y_data[i], z_data[i]);
    }
    fclose(pFile);

    free(x_data);
    free(y_data);
    free(z_data);
    free(time_data);

    printf("Done\n");
    return 0;
}