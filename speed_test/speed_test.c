/* ********************************************************************
 *
 *  Copyright (c) 2018 Christian Haettich [feddischson@gmail.com]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*
 * This small hacky tool writes byte(s) to a serial device and reads
 * it back. The time is measured, a histogramm as well as the
 * mean/min/max value is calculated
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/** @brief Width of the Histogram */
#define HIST_WIDTH 40

void print_usage(void) {
   printf("Usage:\n");
   printf("speed_test <dev> <baud> <cnt> <init>\n");
   printf("       <dev>       Serial device\n");
   printf("       <baud>      Baud-rate of <dev>\n");
   printf("       <cnt>       Number of write/read iterations\n");
   printf("       <init>      Initial counter\n");
   printf("       <blk_size>  Block size, 0 means random midi messages\n");
}

void setup_port(int fd, int baud, int parity);
void setup_port_blocking(int fd, int blocking_state, int blk_size);
double run_timing_test(int fd, int cnt, int init, int blk_size, int n_bin,
                       double bin_size, int midi);
void print_usage(void);
void print_bin(int n, int bin_max, int size);

int main(int argc, char* argv[]) {
   int fd;
   int baud;
   int cnt;
   int init;
   int blk_size;
   double result;

   if (argc != 6) {
      printf("Error: Wrong number of arguments\n\n");
      print_usage();
      exit(-1);
   }

   fd = open(argv[1], O_RDWR | O_NOCTTY | O_SYNC);
   if (fd < 0) {
      perror("Failed to open serial port");
      exit(-1);
   }

   baud = atoi(argv[2]);
   cnt = atoi(argv[3]);
   init = atoi(argv[4]);
   blk_size = atoi(argv[5]);
   setup_port(fd, baud, 0);

   printf("Opened %s @ %d\n", argv[1], baud);
   if (blk_size == 0) {
      result = run_timing_test(fd, cnt, init, 3, 5, 1.0000, 1);
   } else {
      result = run_timing_test(fd, cnt, init, blk_size, 5, 1.0000, 0);
   }
   printf("result: %f s\n", result / CLOCKS_PER_SEC);

   close(fd);
}

int create_random_midi(uint8_t* buf) {
   buf[0] = (8 + (rand() % 8)) << 4;
   if (buf[0] == 0xf0) {
      buf[0] |= rand() % 8;
      buf[1] = rand() & 0xff;
      return 2;
   } else if (buf[0] == 0xc0 || buf[0] == 0xd0) {
      buf[0] |= rand() & 0xf;
      buf[1] = rand() & 0xff;
      return 2;
   } else {
      buf[0] |= rand() & 0xf;
      buf[1] = rand() & 0xff;
      buf[2] = rand() & 0xff;
      return 3;
   }
   return 0;
}

void read_and_dump(int fd) {
   char buf[256];
   printf("\n\n");
   setup_port_blocking(fd, 1, 1);
   while (1) {
      memset(buf, 0, 256);
      int received = read(fd, buf, 256);
      printf("%s", buf);
   }
}

double run_timing_test(int fd, int cnt, int init, int blk_size, int n_bin,
                       double bin_size, int midi) {
   int i_iter;
   int i_bin;
   uint8_t* buf_out;
   uint8_t* buf_in;
   int fail_cnt = 0;
   double mean = 0.0;
   double* timing;
   int* bins;
   double max;
   double min;

   int n_bin_ds;
   double bin_max = 0;

   /* The argument `n_bin` is on-sided (without the center). */
   /* -> make it odd and two-sided (including the center).*/
   n_bin_ds = 2 * n_bin + 1;

   timing = (double*)malloc(cnt * sizeof(double));
   if (timing == 0) {
      perror("Failed to allocate memory for timing measures.");
      goto do_exit1;
   }

   bins = (int*)malloc(n_bin_ds * sizeof(int));
   if (bins == 0) {
      perror("Failed to allocate memory for histogram.");
      goto do_exit2;
   }

   buf_out = (uint8_t*)malloc(blk_size * sizeof(uint8_t));
   if (buf_out == 0) {
      perror("Failed to allocate memory for output buffer.");
      goto do_exit3;
   }

   buf_in = (uint8_t*)malloc(blk_size * sizeof(uint8_t));
   if (buf_in == 0) {
      perror("Failed to allocate memory for input buffer.");
      goto do_exit4;
   }

   /* **************
    *
    * Clear input buffer
    *
    */
   setup_port_blocking(fd, 0, blk_size);
   while (1) {
      if (blk_size != read(fd, buf_in, blk_size)) {
         break;
      }
   }
   setup_port_blocking(fd, 1, blk_size);

   /* ***************
    *
    * Write - read operation and time measurement
    *
    * */

   for (i_iter = 0; i_iter < cnt; ++i_iter) {
      double result;
      struct timespec start = {0, 0};
      struct timespec stop = {0, 0};
      int msg_len = blk_size;
      int received = 0;

      if (midi) {
         msg_len = create_random_midi(buf_out);
      } else {
         memset(buf_out, (char)((init + i_iter) & 0xff), blk_size);
      }

      clock_gettime(CLOCK_MONOTONIC, &start);
      write(fd, buf_out, msg_len);
      received = read(fd, buf_in, msg_len);
      if (msg_len != received) {
         printf("Failed to read %d != %d\n", received, msg_len);
         goto do_exit5;
      } else {
         clock_gettime(CLOCK_MONOTONIC, &stop);
      }
      for (int i = 0; i < msg_len; i++) {
         if (buf_in[i] != (buf_out[i])) {
            printf("%x - %x\n", buf_in[i], buf_out[i]);
            fail_cnt++;
         }
      }

      /* calculate result in milli-seconds */
      result = (stop.tv_sec - start.tv_sec) * 1e3 +
               (stop.tv_nsec - start.tv_nsec) / 1e6;
      timing[i_iter] = result;
      mean += result;
   }
   mean /= (double)cnt;

   printf("fail_cnt = %d\n", fail_cnt);
   printf("mean: %f (ms)\n", mean);

   double hist_start;
   double hist_end;

   /* ************
    *
    * calculate the statistics
    *
    * */
   memset(bins, 0, n_bin_ds * sizeof(int));
   /* upper limit of the lowest bin */
   hist_start = mean - bin_size / 2.0 - (double)(n_bin - 1) * bin_size;

   /* lower-limit of the highest bin */
   hist_end = mean + bin_size / 2.0 + (double)(n_bin - 1) * bin_size;

   for (i_iter = 0; i_iter < cnt; ++i_iter) {
      if (i_iter == 0) {
         max = timing[i_iter];
         min = timing[i_iter];
      } else {
         if (timing[i_iter] > max) {
            max = timing[i_iter];
         }
         if (timing[i_iter] < min) {
            min = timing[i_iter];
         }
      }

      /* check if value is in the lowest bin */
      if (timing[i_iter] < hist_start) {
         bins[0]++;

         /* check if value is in the highest bin */
      } else if (timing[i_iter] >= hist_end) {
         bins[n_bin_ds - 1]++;

         /* the value must be somewhere in the middle */
      } else {
         int found = 0;
         for (i_bin = 0; i_bin < (n_bin_ds - 2); ++i_bin) {
            double low = hist_start + (double)i_bin * bin_size;
            double high = low + bin_size;
            if (timing[i_iter] >= low && timing[i_iter] < high) {
               bins[i_bin + 1]++;
               found = 1;
               break;
            }
         }

         if (0 == found) {
            printf("Enternal runtime error (histogramm calculation)\n");
            goto do_exit6;
         }
      }
   }
   /* **********
    *
    *  print the histogram
    *
    * */

   /* get the maximum size of the bins */
   for (i_bin = 0; i_bin < n_bin_ds - 1; i_bin++) {
      if (bins[i_bin] > bin_max) {
         bin_max = (double)bins[i_bin];
      }
   }

   printf("min: %f (ms)\nmax: %f(ms)\n", min, max);
   printf("\n\n");
   printf("------------------\n");
   print_bin(bins[0], bin_max, HIST_WIDTH);
   for (i_bin = 0; i_bin < n_bin_ds - 2; ++i_bin) {
      double low = hist_start + (double)i_bin * bin_size;
      printf("% 9.4f (ms)\n", low);
      print_bin(bins[i_bin + 1], bin_max, HIST_WIDTH);
   }
   printf("% 9.4f (ms)\n", hist_end);
   print_bin(bins[n_bin_ds - 1], bin_max, HIST_WIDTH);
   printf("------------------\n");

   free(buf_in);
   free(buf_out);
   free(bins);
   free(timing);
   return mean;

do_exit6:
do_exit5:
   free(buf_in);
do_exit4:
   free(buf_out);
do_exit3:
   free(bins);
do_exit2:
   free(timing);
do_exit1:
   exit(-1);
}

/** @brief Prints bin (fills with '#')
 */
void print_bin(int n, int bin_max, int size) {
   int width = (double)n / (double)bin_max * (double)size;
   printf("\t");
   for (int i = 0; i < width; i++) {
      printf("#");
   }
   printf(" %d \n", n);
}

/**
 * @brief Setups blocking behaviour
 * @param blocking_state 1 = blocking, 0 = non-blocking
 * @param blk_size Number of expected bytes
 */
void setup_port_blocking(int fd, int blocking_state, int blk_size) {
   struct termios tty;
   memset(&tty, 0, sizeof tty);
   if (tcgetattr(fd, &tty) != 0) {
      perror("Failed to get attributes");
      exit(-1);
   }
   if (blocking_state) {
      tty.c_cc[VMIN] = blk_size;
      tty.c_cc[VTIME] = 0;
   } else {
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
   }
   if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("Failed to set attributes");
      exit(-1);
   }
}

/** @brief Configures a serial port
 *  @param fd The file descriptor of the serial port
 *  @param baud The baud rate
 *  @param parity The parity
 */
void setup_port(int fd, int baud, int parity) {
   struct termios tty;
   memset(&tty, 0, sizeof tty);
   if (tcgetattr(fd, &tty) != 0) {
      perror("Failed to get attributes");
      exit(-1);
   }

   cfsetospeed(&tty, baud);
   cfsetispeed(&tty, baud);
   tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
   tty.c_iflag &= ~IGNBRK;
   tty.c_lflag = 0;
   tty.c_oflag = 0;
   tty.c_cc[VMIN] = 0;
   tty.c_cc[VTIME] = 100;
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);
   tty.c_cflag |= (CLOCAL | CREAD);
   tty.c_cflag &= ~(PARENB | PARODD);
   tty.c_cflag |= parity;
   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CRTSCTS;

   if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("Failed to set attributes");
      exit(-1);
   }
}
