/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/
#include "sensor.h"

//#include <MPU9250_6Axis_MotionApps_V6_12.h>
#include "MPU9250k.h"

// @formatter:off
template <typename T> struct vector3
{
  T x,y,z;

  vector3() {x=y=z=0; }
  vector3(T _x,T _y,T _z) { x=_x; y=_y; z=_z; }
  template <typename U> vector3(const vector3<U> & i) { x = (T)i.x; y = (T)i.y; z = (T)i.z;  }
  template <typename U> vector3(U * i)  {    x = (T)i[0];    y = (T)i[1];    z = (T)i[2];  }

  T* toArray() { return reinterpret_cast<T*>(this); }

  const vector3<T> operator*(const T k) const { return vector3<T>(x * k, y * k, z * k); }
  const vector3<T> operator/(const T k) const { return vector3<T>(x / k, y / k, z / k); }
  const vector3<T> operator+(const vector3<T> a) const { return vector3<T>(x + a.x, y + a.y, z + a.z); }
  const vector3<T> operator-(const vector3<T> a) const { return vector3<T>(x - a.x, y - a.y, z - a.z); }
  float operator*(const vector3<T> b) const { return x * b.x + y * b.y + z * b.z; }

  const vector3<T> norm() const { return *this / sqrtf(*this * *this);  }
};
// @formatter:on


class MPU9250KSensor : public Sensor
{
public:
    MPU9250KSensor(){};
    ~MPU9250KSensor(){};
    void motionSetup() override final;
    void motionLoop() override final;
    void startCalibration(int calibrationType) override final;
    void getMPUScaled();

private:
    MPU9250K imu{99};
    CalibrationConfig *calibration;
    //bool dmpReady = false;    // set true if DMP init was successful
    //uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
    //uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
    //uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
    //uint16_t fifoCount;       // count of all bytes currently in FIFO
    //uint8_t fifoBuffer[64]{}; // FIFO storage buffer
    //raw data and scaled as vector
    //int skipCalcMag = 0;
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // for raw filter
    //float Axyz[3]{};
    //float Gxyz[3]{};
    //float Mxyz[3]{};
    //float rawMag[3]{};
    //Quat correction{0,0,0,0};
    // Loop timing globals
    unsigned long now = 0, last = 0; //micros() timers
    float deltat = 0;                //loop time in seconds

    const uint8_t Gscale = GFS_250DPS, Ascale = AFS_4G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;
    float aRes, gRes, mRes;
    float magCalibration[3] = {0, 0, 0};
    int16_t MPU9250Data[7];
    int16_t magCount[3];

    vector3<float> gyroBias;
    vector3<float> magBias;


    void updateAG() { imu.readMPU9250Data(addr, MPU9250Data); }
    vector3<float> getAccel() { return vector3<float>(&MPU9250Data[0]) * aRes; }
    vector3<float> getGyroRaw() { return vector3<float>(&MPU9250Data[4]) * gRes;   }
    vector3<float> getGyro()   {   return getGyroRaw() - gyroBias;   }

    void updateM() {   imu.readMagData(addr, magCount);  }
    vector3<float> getMagRaw() { return vector3<float>(&magCount[0]); }
    vector3<float> getMag() { return getMagRaw() - magBias; }
};