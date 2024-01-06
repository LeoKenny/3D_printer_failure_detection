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

const double conversion_const = 0.004;              // Max resolution, fixed to 4mg/LSB, pg 27
const int rw_bit = 7;                               // Read/Write bit
const int multi_byte_bit = 6;                       // Mutiple byte bit
const int fifo_size = 33;                           // Max number of saved values, FIFO + last reading

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
    //      0    |  0  |      0     | 0 |     1    |    0    |  0 0  |
    //     OFF   |4-wir|active high | 0 |  13-bit  | right-j | +- 2g |
    // (datasheet ADXL345, pg 27)
    spi_write(spi_handle, command, 2);
    delay_ms(1);

    // Set Sample Rate
    command[0] = BW_RATE;
    // command[1] = 0x0F;
    command[1] = 0x07;
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
    // command[1] = 0xB0;
    command[1] = 0xA1;
    // FIFO_MODE | Trigger | Samples
    //    1 0    |    1    | 1 0 0 0 0
    //  stream   |   INT2  | 16 samples
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

void save_data(char *output_name, int samples_counter, int *overrun_data,
               unsigned long int *number_data, double *block_time_data, double *sample_time_data,
               double *x_data, double *y_data, double *z_data){
    // Save data
    FILE * pFile;
    pFile = fopen(output_name, "a");
    for(int i=0; i < samples_counter; i++){
        fprintf(pFile, "%ld, %.9f, %.9f, %d, %.9f, %.9f, %.9f\n",
                number_data[i], block_time_data[i], sample_time_data[i],
                overrun_data[i], x_data[i], y_data[i], z_data[i]);
    }
    fclose(pFile);
}

// int data_acquisition(struct timespec start_time, int sampled_values, int spi_handle,
//                       int overrun_trigger,
//                       unsigned long *sample_number,
//                       unsigned long *number_data, int *overrun_data,
//                       double *block_time_data, double *sample_time_data,
//                       double *x_data, double *y_data, double *z_data){
//     char command[8], buffer[8];
//     int result;
//     int samples_counter = 0;
//     double x,y,z,t_block,t_sample;

//     t_block = time_delta_now(start_time);
//     for(int i=0; i<sampled_values; i++,delay_ms(0.006)){
//         command[0] = DATAX0;
//         result = spi_read(spi_handle, command, buffer, spi_buffer_size);
//         if(result < spi_buffer_size){
//             printf("Buffer read size {%d} is smaller than expected {%d}.\n", result, spi_buffer_size);
//         }
//         else{
//             t_sample = time_delta_now(start_time);
//             x = (buffer[2]<<8) | buffer[1];
//             y = (buffer[4]<<8) | buffer[3];
//             z = (buffer[6]<<8) | buffer[5];

//             number_data[samples_counter] = *sample_number;
//             block_time_data[samples_counter] = t_block;
//             sample_time_data[samples_counter] = t_sample;
//             overrun_data[samples_counter] = overrun_trigger;
//             x_data[samples_counter] = x; //*conversion_const;
//             y_data[samples_counter] = y; //*conversion_const;
//             z_data[samples_counter] = z; //*conversion_const;

//             samples_counter++;
//             *sample_number++;
//         }
//     }
//     return samples_counter;
// }

int main(int argc, char *argv[]) {
    char buffer[spi_buffer_size];
    char command[spi_buffer_size];
    int spi_handle;
    int result, sampled_values, watermark_trigger, overrun_trigger;
    double t_sample, t_block;
    int16_t x, y, z;
    unsigned long int *number_data;
    double *block_time_data, *sample_time_data;
    int *overrun_data;
    double *x_data, *y_data, *z_data;
    struct timespec start_time;

    int samples_counter = 0;                // counter of samples
    double sample_time = 50;                 // sample time in seconds
    int sample_rate = 3200;                 // sample rate in Hz
    unsigned long int sample_number = 0;    // Samples total number to sort order
    char output_name[256] = "data.csv";

    // Allocate space for data
    number_data = malloc(fifo_size * sizeof(unsigned long int));
    overrun_data = malloc(fifo_size * sizeof(int));
    x_data = malloc(fifo_size * sizeof(double));
    y_data = malloc(fifo_size * sizeof(double));
    z_data = malloc(fifo_size * sizeof(double));
    block_time_data = malloc(fifo_size * sizeof(double));
    sample_time_data = malloc(fifo_size * sizeof(double));

    // Create Output file, and put header
    FILE * pFile;
    pFile = fopen(output_name, "w");
    fprintf(pFile, "sample_number, block_time, sample_time, overrun, x, y, z\n");
    fclose(pFile);

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

    clear_fifo_forced(spi_handle);
    delay_ms(1/(double)sample_rate);

    clock_gettime(CLOCK_REALTIME, &start_time);

    while(time_delta_now(start_time) < sample_time){
        sampled_values = fifo_status(spi_handle);
        trigger_status(spi_handle, &watermark_trigger, &overrun_trigger);

        if(overrun_trigger){
            printf("Overrun!");
            printf("Delta: %f\n", time_delta_now(start_time));
        }

        if(watermark_trigger>0){
            // samples_counter = data_acquisition(start_time, sampled_values, spi_handle, overrun_trigger,
            //                  &sample_number, &number_data, &overrun_data,
            //                  &block_time_data, &sample_time_data,
            //                  &x_data, &y_data, &z_data);
            // overrun_trigger = 0;
            t_block = time_delta_now(start_time);
            for(int i=0; i<sampled_values; i++,delay_ms(0.006)){
                command[0] = DATAX0;
                result = spi_read(spi_handle, command, buffer, spi_buffer_size);
                if(result < spi_buffer_size){
                    printf("Buffer read size {%d} is smaller than expected {%d}.\n", result, spi_buffer_size);
                }
                else{
                    t_sample = time_delta_now(start_time);
                    x = (buffer[2]<<8) | buffer[1];
                    y = (buffer[4]<<8) | buffer[3];
                    z = (buffer[6]<<8) | buffer[5];

                    number_data[samples_counter] = sample_number;
                    block_time_data[samples_counter] = t_block;
                    sample_time_data[samples_counter] = t_sample;
                    overrun_data[samples_counter] = overrun_trigger;
                    x_data[samples_counter] = x; //*conversion_const;
                    y_data[samples_counter] = y; //*conversion_const;
                    z_data[samples_counter] = z; //*conversion_const;

                    printf("X: %f, Y: %f, Z: %f\n", x_data[samples_counter],y_data[samples_counter],z_data[samples_counter]);

                    samples_counter++;
                    sample_number++;
                    overrun_trigger = 0;
                }
            }
        }
        if(samples_counter > 0){
            save_data(output_name, samples_counter, overrun_data,
                      number_data, block_time_data, sample_time_data,
                      x_data, y_data, z_data);
            samples_counter = 0;
        }
        delay_ms(MS_PER_SECOND/((double)sample_rate*2));
    }

    printf("\nElapsed Time: %.6f seconds\n", time_delta_now(start_time));
    printf("Samples Quantity: %d\n", samples_counter);

    // Finishing SPI and GPIO
    spiClose(spi_handle);
    gpioTerminate();

    free(x_data);
    free(y_data);
    free(z_data);
    free(number_data);
    free(block_time_data);
    free(sample_time_data);
    free(overrun_data);

    printf("Done\n");
    return 0;
}