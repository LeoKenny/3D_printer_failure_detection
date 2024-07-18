#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

#include <pigpio.h>

const double conversion_const = (2.0*16.0)/8192;       // +-16g for 13 bits, pg 27

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

int spi_write_and_read(int spi_handle, char *command, char *buffer, int count){
    return spiXfer(spi_handle, command, buffer, count);
}

int spi_read(int spi_handle, char *buffer, int count){
    return spiRead(spi_handle,buffer,count);
}

int spi_write(int spi_handle, char *buffer, int count){
    return spiWrite(spi_handle, buffer, count);
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


int main(int argc, char *argv[]){
    char buffer[8] = {1,2,3,4,5,6,7,8};
    char input[8];
    int spi_handle;
    // Starting GPIO
    if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
        printf("GPIO initialization failed.");
        return 1;
    }

    // Starting SPI
    spi_handle = spiOpen(spi_channel, spi_speed, 0);
    if(verify_spi(spi_handle) < 0){ return 1; }
    
    delay_ms(2*MS_PER_SECOND);

    spi_write_and_read(spi_handle,buffer,input,8);

    delay_ms(2*MS_PER_SECOND);

    for (int i=0; i<8; i++){
        printf("%d ", input[i]);
    }
    printf("\n");

    spiClose(spi_handle);
    return 0;
}

// int main(int argc, char *argv[]) {
//     char buffer[spi_buffer_size];
//     char command[spi_buffer_size];
//     int spi_handle;
//     int result, sampled_values, watermark_trigger, overrun_trigger;
//     double t_sample, t_block;
//     int16_t x, y, z;
//     unsigned long int *number_data;
//     double *block_time_data, *sample_time_data;
//     int *overrun_data;
//     double *x_data, *y_data, *z_data;
//     struct timespec start_time;
//
//     int samples_counter = 0;                // counter of samples
//     double sample_time = 3;                 // sample time in seconds
//     int sample_rate = 3200;                 // sample rate in Hz
//     unsigned long int sample_number = 0;    // Samples total number to sort order
//     char default_output_name[] = "data.csv";
//     char vectors[6][3] = {"x1","x2","y1","y2","z1","z2"};
//     char output_name[256];
//
//     // Allocate space for data
//     number_data = malloc(fifo_size * sizeof(unsigned long int));
//     overrun_data = malloc(fifo_size * sizeof(int));
//     x_data = malloc(fifo_size * sizeof(double));
//     y_data = malloc(fifo_size * sizeof(double));
//     z_data = malloc(fifo_size * sizeof(double));
//     block_time_data = malloc(fifo_size * sizeof(double));
//     sample_time_data = malloc(fifo_size * sizeof(double));
//
//
//     // Starting GPIO
//     if (gpioInitialise() == PI_INIT_FAILED){      // pigpio initialisation failed.
//         printf("GPIO initialization failed.");
//         return 1;
//     }
//
//     // Starting SPI
//     spi_handle = spiOpen(spi_channel, spi_speed, 3);
//     if(verify_spi(spi_handle) < 0){ return 1; }
//
//     configure_adxl(spi_handle);
//
//     for(int i=0; i<6; i++){
//         strcpy(output_name, vectors[i]);
//         strcat(output_name, "_");
//         strcat(output_name, default_output_name);
//
//         printf("Click any character to start aquisition for %s, data saved in %s", vectors[i], output_name);
//         getchar();
//
//         // Create Output file, and put header
//         FILE * pFile;
//         pFile = fopen(output_name, "w");
//         fprintf(pFile, "sample_number, block_time, sample_time, overrun, x, y, z\n");
//         fclose(pFile);
//
//
//         // Starting acquisition
//         printf("Sample Time: %.6f seconds\n",sample_time);
//         clear_fifo_forced(spi_handle);
//         delay_ms(1/(double)sample_rate);
//         clock_gettime(CLOCK_REALTIME, &start_time);
//         while(time_delta_now(start_time) < sample_time){
//             sampled_values = fifo_status(spi_handle);
//             trigger_status(spi_handle, &watermark_trigger, &overrun_trigger);
//
//             if(overrun_trigger){
//                 printf("Overrun!");
//                 printf("Delta: %f\n", time_delta_now(start_time));
//             }
//
//             if(watermark_trigger>0){
//                 t_block = time_delta_now(start_time);
//                 for(int i=0; i<sampled_values; i++,delay_ms(0.006)){
//                     command[0] = DATAX0;
//                     result = spi_read(spi_handle, command, buffer, spi_buffer_size);
//                     if(result < spi_buffer_size){
//                         printf("Buffer read size {%d} is smaller than expected {%d}.\n", result, spi_buffer_size);
//                     }
//                     else{
//                         t_sample = time_delta_now(start_time);
//                         x = (buffer[2]<<8) | buffer[1];
//                         y = (buffer[4]<<8) | buffer[3];
//                         z = (buffer[6]<<8) | buffer[5];
//
//                         number_data[samples_counter] = sample_number;
//                         block_time_data[samples_counter] = t_block;
//                         sample_time_data[samples_counter] = t_sample;
//                         overrun_data[samples_counter] = overrun_trigger;
//                         x_data[samples_counter] = (double)x * conversion_const;
//                         y_data[samples_counter] = (double)y * conversion_const;
//                         z_data[samples_counter] = (double)z * conversion_const;
//
//                         samples_counter++;
//                         sample_number++;
//                         overrun_trigger = 0;
//                     }
//                 }
//             }
//             if(samples_counter > 0){
//                 save_data(output_name, samples_counter, overrun_data,
//                         number_data, block_time_data, sample_time_data,
//                         x_data, y_data, z_data);
//                 samples_counter = 0;
//             }
//             delay_ms(MS_PER_SECOND/((double)sample_rate*2));
//         }
//     }
//
//     // Finishing SPI and GPIO
//     spiClose(spi_handle);
//     gpioTerminate();
//
//     free(x_data);
//     free(y_data);
//     free(z_data);
//     free(number_data);
//     free(block_time_data);
//     free(sample_time_data);
//     free(overrun_data);
//
//     printf("Done\n");
//     return 0;
// }