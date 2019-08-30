/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

Changed by O.bordes August 2019, add teapot output data

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/
#include <SparkFunMPU9250-DMP.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
#define OUTPUT_TEAPOT

void setup() 
{
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              10); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();
      imu.update(UPDATE_COMPASS);
      printIMUData();
    }
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
  float q0 = imu.calcQuat(imu.qw);
  float q1 = imu.calcQuat(imu.qx);
  float q2 = imu.calcQuat(imu.qy);
  float q3 = imu.calcQuat(imu.qz);


#ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:

           // q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
           // q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
           // q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
           // q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
                
            teapotPacket[2] = (imu.qw >> 24) & 0xff; 
            teapotPacket[3] = (imu.qw >> 16) & 0xff;
            teapotPacket[4] = (imu.qx >> 24) & 0xff; 
            teapotPacket[5] = (imu.qx >> 16) & 0xff;
            teapotPacket[6] = (imu.qy >> 24) & 0xff; 
            teapotPacket[7] = (imu.qy >> 16) & 0xff;
            teapotPacket[8] = (imu.qz >> 24) & 0xff; 
            teapotPacket[9] = (imu.qz >> 16) & 0xff;
            SerialPort.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        
#else


  SerialPort.println("Q: " + String(q0, 4) + ", " +
                    String(q1, 4) + ", " + String(q2, 4) + 
                    ", " + String(q3, 4));
  SerialPort.println("qX: " + String(imu.qw) + ", " +
                    String(imu.qx) + ", " + String(imu.qy) + 
                    ", " + String(imu.qz));
  SerialPort.println("R/P/Y: " + String(imu.roll) + ", "
            + String(imu.pitch) + ", " + String(imu.yaw));
  SerialPort.println("Heading: " + String(imu.computeCompassHeading()));
  SerialPort.println("Time: " + String(imu.time) + " ms");
  SerialPort.println();
  #endif
}
