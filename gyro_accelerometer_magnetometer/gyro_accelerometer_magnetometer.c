/*
This program reads the angles from the accelerometer and gyroscope
and the magnetometer data on a BerryIMU connected to a Raspberry Pi.

A Kalman filter is used on the accelerometer and gyroscope data.

Based on gyro_accelerometer_tutorial03_kalman_filter and compass_tutorial_01.
*/

#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include "IMU.c"


#define DT 0.02         // [s/loop] loop period. 20ms

#define G_GAIN 0.070     // [deg/s/LSB]

void  INThandler(int sig)// Used to do a nice clean exit when Ctrl-C is pressed
{
  signal(sig, SIG_IGN);
  exit(0);
}

int mymillis()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
  long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
  result->tv_sec = diff / 1000000;
  result->tv_usec = diff % 1000000;
  return (diff<0);
}

int main(int argc, char *argv[])
{
  float rate_gyr_y = 0.0;   // [deg/s]
  float rate_gyr_x = 0.0;   // [deg/s]
  float rate_gyr_z = 0.0;   // [deg/s]

  int  accRaw[3];
  int  magRaw[3];
  int  gyrRaw[3];

  float AccYangle = 0.0;
  float AccXangle = 0.0;

  float accX = 0.0;
  float accY = 0.0;
  float accZ = 0.0;

  int startInt  = mymillis();
  struct  timeval tvBegin, tvEnd,tvDiff;

  signal(SIGINT, INThandler);

  detectIMU();
  enableIMU();

  gettimeofday(&tvBegin, NULL);

  //printf("Loop Time 0\t");
  printf("LoopTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,MagRawX,MagRawY,MagRawZ\n");
  printf("0,");
  while(1)
  {
    startInt = mymillis();

    //read ACC and GYR data
    readACC(accRaw);
    readGYR(gyrRaw);

    //Convert Gyro raw to degrees per second
    rate_gyr_x = (float) gyrRaw[0]  * G_GAIN;
    rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
    rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

    accX = accRaw[0];
    accY = accRaw[1];
    accZ = accRaw[2];

    //read magnetometer data
    readMAG(magRaw);

    printf("%.3f,%.3f,%.3f,",rate_gyr_x,rate_gyr_y,rate_gyr_z);
    printf("%.3f,%.3f,%.3f,",accX,accY,accZ);
    printf("%i,%i,%i\n",magRaw[0],magRaw[1],magRaw[2]);

    //Each loop should be at least 20ms.
    while(mymillis() - startInt < (DT*1000)){
      usleep(100);
    }

    //printf("Loop Time %d\t", mymillis()- startInt);
    printf("%d,", mymillis()- startInt);
  }
}
