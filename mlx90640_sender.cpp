#include <stdint.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#include <thread>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "MLX90640_API.h"
#include "datalink/network.h"

#define MLX_I2C_ADDR 0x33

#define SENSOR_W 24
#define SENSOR_H 32

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// The i2c baudrate is set to 1mhz to support these
#define FPS 32

#define LOOP_HZ 100

bool running = true;

timespec diff( timespec start, timespec end ) {
    timespec temp;
    if( ( end.tv_nsec - start.tv_nsec ) < 0 ) {
        temp.tv_sec  =              end.tv_sec  - start.tv_sec  - 1;
        temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    } else {
        temp.tv_sec  =              end.tv_sec  - start.tv_sec;
        temp.tv_nsec =              end.tv_nsec - start.tv_nsec;
    }
    return temp;
}

void standard_dev(const float* val, float& stan_dev, float& high_stan_dev) {
	float sum = 0.0f;
    float sum_square = 0.0f;

    for (int i = 0; i < 768; i++) {
        sum += val[i];
	}
	float mean = sum / 768;

    for (int i = 0; i < 768; i++) {
        sum_square += pow(val[i] - mean, 2);
    }
	stan_dev = sqrt(sum_square / 768);

    if (stan_dev > high_stan_dev) {
        high_stan_dev = stan_dev;
	}
}

    struct Segment {
        int start;
        int end;
    };

    struct Result {
        int row;
        Segment seg_num[2];
    };

int two_pass(const float* frame, Result* results) {
    int row_count = 0;

    for (int y = 0; y < SENSOR_W; y++) {
        Segment seg_noise[3];
        int seg_index = 0;
        bool in_segment = false;

        for (int x = 0; x < SENSOR_H; x++) {
            float val = frame[SENSOR_H * y + (SENSOR_H - 1 - x)];
            //bool hot = val > 50.0f; // Threshold for "hot" pixels, adjust as needed
            bool hot = val > 30.0f; // Threshold for "hot" pixels, adjust as needed
            if (hot && !in_segment) {
                // Start of a new segment
                if (seg_index < 3) {
                    seg_noise[seg_index].start = x;
                    seg_noise[seg_index].end = x;
                }
                in_segment = true;
            } else if (hot && in_segment) {
                seg_noise[seg_index].end = x;
            } else if (!hot && in_segment) {
                // End of the current segment
                in_segment = false;
                seg_index++;
            }
        }

        if (in_segment) seg_index++; // Close any open segment at the end of the row
        if (seg_index == 2 && row_count < 24) {
            results[row_count].row = y;
            results[row_count].seg_num[0] = seg_noise[0];
            results[row_count].seg_num[1] = seg_noise[1];
            row_count++;
        }
    }
    return row_count;
}

int frame_centroid(const float* frame) {
	Result results[24];
	int valid_rows = two_pass(frame, results);
	int valid_col = two_pass(frame, results);
	float col_centroid = 0.0f;
	float row_centroid = 0.0f;

    for (int i = 0; i < valid_rows; i++) {
        int row = results[i].row;
        for (int j = 0; j < 2; j++) {
            int start = results[i].seg_num[j].start;
            int end = results[i].seg_num[j].end;
            if (end > start) {
                valid_col++;
                col_centroid += (start + end) / 2.0f;
                row_centroid += row;
            }
        }
	}
    if (valid_col > 0) {
        col_centroid /= valid_col;
        row_centroid /= valid_rows;
    }
    printf("Centroid: (%.3f, %.3f)\n", col_centroid, row_centroid);
    // if (col_centroid == 0.0f && row_centroid == 0.0f) printf("Centroid: (%.3f), (%.3f)\n", 999.0f, 999.0f);
    // else printf("Centroid: (%.3f, %.3f)\n", col_centroid, row_centroid);
    //std::cout << "Centroid: (" << col_centroid << ", " << row_centroid << ")\n";
    return 1;
}

char temp_to_char(float val, int minTemp, int maxTemp)
{
    // static const char ramp[] = {'1','2','3','4','5'};  // cold → hot
    static const char ramp[] = {'.',';','&','#','@'};  // cold → hot
    const int n = sizeof(ramp) - 1;

    if (maxTemp <= minTemp)
        return ramp[n / 2];

    float t = (val - minTemp) / (maxTemp - minTemp);

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // >>> non-linear contrast boost <<<
    float gamma = 0.8f;            // adjust to taste (0.4 → strong, 0.8 → subtle)
    t = std::pow(t, gamma);

    int idx = static_cast<int>(t * (n - 1) + 0.5f);
    return ramp[idx];
}


int main(void) {

//     timespec newtime, oldtime;
// 	int startSec;
//     int countThermalCam = 0;
//     // float stan_dev, high_stan_dev = 0;
//     float stan_dev = 0;
//     float high_stan_dev = 0;

// 	int minTemp = 100;
//     int maxTemp = 0;

//     char text_buffer[36];
//     static uint16_t eeMLX90640[832];
//     float emissivity = 1;
//     uint16_t frame[834];
//     static float mlx90640To[768];
//     double sendFrame[768];
//     float eTa;

//     int count_frames = 0;

//     /* get network ready */
// 	openPort();

//     printf("Starting set up...\n");

//     clock_gettime( CLOCK_REALTIME, &oldtime );
// 	startSec = oldtime.tv_sec;

//     switch(FPS){
//         case 1:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b001);
//             break;
//         case 2:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b010);
//             break;
//         case 4:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b011);
//             break;
//         case 8:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b100);
//             break;
//         case 16:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b101);
//             break;
//         case 32:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b110);
//             break;
//         case 64:
//             MLX90640_SetRefreshRate(MLX_I2C_ADDR, 0b111);
//             break;
//         default:
//             printf("Unsupported framerate: %d", FPS);
//             return 1;
//     }
//     MLX90640_SetChessMode(MLX_I2C_ADDR);
//     printf("Selected chess mode\n");

//     paramsMLX90640 mlx90640;
//     MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
//     MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
//     printf("Going for while\n");
    
//     while( true ){
//         clock_gettime( CLOCK_REALTIME, &newtime );
// 		if( diff( oldtime, newtime ).tv_nsec > (1000000000/LOOP_HZ) ) {
// 			oldtime.tv_nsec += (1000000000/LOOP_HZ);
// 			if( oldtime.tv_nsec > 1000000000 ) {
// 				oldtime.tv_sec--;
// 				oldtime.tv_nsec -= 1000000000;
// 			}
// 			countThermalCam++;

//             if ( 5 == (countThermalCam%10) ) {
//                 count_frames++;
//                 MLX90640_GetFrameData(MLX_I2C_ADDR, frame);

//                 eTa = MLX90640_GetTa(frame, &mlx90640);
//                 MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);

//                 MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);
//                 MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640);

//                 // standard_dev(mlx90640To,stan_dev, high_stan_dev);
// 				// frame_centroid(mlx90640To);

//                 /* send data to onboard */
//                 for(int i = 0; i < SENSOR_W; i++){
//                     for(int j = 0; j < SENSOR_H; j++){
//                         sendFrame[SENSOR_H * i + j] = (double)mlx90640To[SENSOR_H * i + (SENSOR_H - 1 - j)];
//                     }
//                 }

//                 sendThermalData( &sendFrame[0] );

//                 minTemp = 100;
//                 maxTemp = 0;
//                 for(int i=0;i<768;i++){
//                     if(minTemp > mlx90640To[i]) minTemp = mlx90640To[i];
//                     if(maxTemp < mlx90640To[i]) maxTemp = mlx90640To[i];
//                 }

//                 for(int y = 0; y < SENSOR_W; y++){
//                     for(int x = 0; x < SENSOR_H; x++){
//                         /* flips image */
//                         // float val = mlx90640To[SENSOR_H * y + (SENSOR_H - 1 - x)];
//                         float val = mlx90640To[SENSOR_H*(SENSOR_W - 1 - y) + x];
//                         char c = temp_to_char(val, minTemp, maxTemp);
//                         putchar(c);
//                     }
//                     putchar('\n'); // end of row
//                 }
//                 printf("\n\n"); // end of frame
//                 // first couple frames are sort of trash
//                 if (count_frames > 1) {
//                     standard_dev(mlx90640To,stan_dev, high_stan_dev);
//                     frame_centroid(mlx90640To);

//                     printf("Frame Num: %d\tMax:%d C\tMin:%d C\t Frame Std_Dev:%.6f\tHigh Std_Dev:%.6f\n",count_frames,maxTemp,minTemp, stan_dev, high_stan_dev);
//                 }
//             }
//         }
//         usleep(5000);  /* release CPU */
//     }
}
